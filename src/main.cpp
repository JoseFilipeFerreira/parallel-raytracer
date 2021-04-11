#include "math/vec.h"
#include "scene/camera.h"
#include "scene/ray_triangle.h"
#include "scene/scene.h"
#include "scene/sceneloader.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>

bool intersect(
    const tracer::scene& SceneMesh,
    const tracer::vec3<float>& ori,
    const tracer::vec3<float>& dir,
    float& t,
    float& u,
    float& v,
    size_t& geomID,
    size_t& primID) {
    for (auto i = 0; i < SceneMesh.geometry.size(); i++) {
        for (auto f = 0; f < SceneMesh.geometry[i].face_index.size(); f++) {
            auto face = SceneMesh.geometry[i].face_index[f];
            if (tracer::intersect_triangle(
                    ori,
                    dir,
                    SceneMesh.geometry[i].vertex[face[0]],
                    SceneMesh.geometry[i].vertex[face[1]],
                    SceneMesh.geometry[i].vertex[face[2]],
                    t,
                    u,
                    v)) {
                geomID = i;
                primID = f;
            }
        }
    }
    return (geomID != -1 && primID != -1);
}

bool occlusion(
    const tracer::scene& SceneMesh,
    const tracer::vec3<float>& ori,
    const tracer::vec3<float>& dir,
    float& t) {
    float u, v;
    for (auto i = 0; i < SceneMesh.geometry.size(); i++) {
        for (auto f = 0; f < SceneMesh.geometry[i].face_index.size(); f++) {
            auto face = SceneMesh.geometry[i].face_index[f];
            if (tracer::intersect_triangle(
                    ori,
                    dir,
                    SceneMesh.geometry[i].vertex[face[0]],
                    SceneMesh.geometry[i].vertex[face[1]],
                    SceneMesh.geometry[i].vertex[face[2]],
                    t,
                    u,
                    v)) {
                return true;
            }
        }
    }
    return false;
}

struct RenderEnv {
    int image_width;
    int image_height;
    std::vector<tracer::vec3<float>>& image;
    tracer::camera& cam;
    tracer::scene& SceneMesh;
};

void calculate_line(RenderEnv render_env, int line) {

    std::uniform_real_distribution<float> distrib(0, 1.f);
    std::random_device rd;
    std::mt19937 gen(rd());

    for (int w = 0; w < render_env.image_width; w++) {
        size_t geomID = -1;
        size_t primID = -1;

        auto is = float(w) / (render_env.image_width - 1);
        auto it = float(line) / (render_env.image_height - 1);
        auto ray = render_env.cam.get_ray(is, it);

        float t = std::numeric_limits<float>::max();
        float u = 0;
        float v = 0;
        if (intersect(render_env.SceneMesh, ray.origin, ray.dir, t, u, v, geomID, primID)) {
            auto i = geomID;
            auto f = primID;
            auto face = render_env.SceneMesh.geometry[i].face_index[f];
            auto N = normalize(cross(
                render_env.SceneMesh.geometry[i].vertex[face[1]] -
                    render_env.SceneMesh.geometry[i].vertex[face[0]],
                render_env.SceneMesh.geometry[i].vertex[face[2]] -
                    render_env.SceneMesh.geometry[i].vertex[face[0]]));

            if (!render_env.SceneMesh.geometry[i].normals.empty()) {
                auto N0 = render_env.SceneMesh.geometry[i].normals[face[0]];
                auto N1 = render_env.SceneMesh.geometry[i].normals[face[1]];
                auto N2 = render_env.SceneMesh.geometry[i].normals[face[2]];
                N = normalize(N1 * u + N2 * v + N0 * (1 - u - v));
            }

            for (auto& lightID : render_env.SceneMesh.light_sources) {
                auto light = render_env.SceneMesh.geometry[lightID];
                light.face_index.size();
                std::uniform_int_distribution<int> distrib1(0, light.face_index.size() - 1);

                int faceID = distrib1(gen);
                const auto& v0 = light.vertex[faceID];
                const auto& v1 = light.vertex[faceID];
                const auto& v2 = light.vertex[faceID];

                auto P = v0 + ((v1 - v0) * float(distrib(gen)) + (v2 - v0) * float(distrib(gen)));

                auto hit = ray.origin + ray.dir * (t - std::numeric_limits<float>::epsilon());
                auto L = P - hit;

                auto len = tracer::length(L);

                t = len - std::numeric_limits<float>::epsilon();

                L = tracer::normalize(L);

                auto mat = render_env.SceneMesh.geometry[i].object_material;
                auto c =
                    (mat.ka * 0.5f + mat.ke) / float(render_env.SceneMesh.light_sources.size());

                if (occlusion(render_env.SceneMesh, hit, L, t)) continue;

                auto d = dot(N, L);

                if (d <= 0) continue;

                auto H = normalize((N + L) * 2.f);

                c = c + (mat.kd * d + mat.ks * pow(dot(N, H), mat.Ns)) /
                            float(render_env.SceneMesh.light_sources.size());

                render_env.image[line * render_env.image_width + w].r += c.r;
                render_env.image[line * render_env.image_width + w].g += c.g;
                render_env.image[line * render_env.image_width + w].b += c.b;
            }
        }
    }
}

class WorkQueue {
    std::deque<std::tuple<size_t, size_t>> queue;
    std::mutex lock;
    std::condition_variable notempty;
    std::atomic_bool sender_closed = false;

  public:
    auto enqueue(std::tuple<size_t, size_t> unit) {
        std::unique_lock<std::mutex> guard(lock);
        queue.push_back(unit);
        notempty.notify_all();
    }
    auto dequeue() -> std::tuple<size_t, size_t> {
        std::unique_lock<std::mutex> guard(lock);
        notempty.wait(guard, [&] { return !queue.empty(); });
        auto v = queue.front();
        queue.pop_front();
        return v;
    }
    auto has_work() -> bool {
        if (!sender_closed) {
            return true;
        } else {
            std::unique_lock<std::mutex> guard(lock);
            return !queue.empty();
        }
    }
    auto close_sender() { sender_closed = true; }
};

class Receiver {
    std::shared_ptr<WorkQueue> work_queue;

  public:
    Receiver(std::shared_ptr<WorkQueue> q): work_queue(q) {}

    auto receive() -> std::optional<std::tuple<size_t, size_t>> {
        if (work_queue->has_work())
            return work_queue->dequeue();
        else
            return {};
    }
};

class Sender {
    std::shared_ptr<WorkQueue> work_queue;

  public:
    Sender(): work_queue(std::shared_ptr<WorkQueue>(new WorkQueue())) {}
    auto send(std::tuple<size_t, size_t> data) { work_queue->enqueue(data); }
    auto make_receiver() -> Receiver { return Receiver(work_queue); };
    ~Sender() { work_queue->close_sender(); };
    Sender(const Sender&) = delete;
    Sender(Sender&& rhs) noexcept: work_queue(std::move(rhs.work_queue)) {}
    Sender& operator=(const Sender&) = delete;
};

void worker(Receiver rx, RenderEnv render_env) {
    while (auto piece = rx.receive()) {
        for (size_t line = std::get<0>(*piece); line < std::get<1>(*piece); line++) {
            calculate_line(render_env, line);
        }
    }
}

int main(int argc, char* argv[]) {
    std::string modelname;
    std::string outputname = "output.ppm";
    bool hasEye{false}, hasLook{false};
    tracer::vec3<float> eye(0, 1, 3), look(0, 1, 0);
    tracer::vec2<uint> windowSize(1024, 768);

    for (int arg = 0; arg < argc; arg++) {
        if (std::string(argv[arg]) == "-m") {
            modelname = std::string(argv[arg + 1]);
            arg++;
            continue;
        }
        if (std::string(argv[arg]) == "-o") {
            outputname = std::string(argv[arg + 1]);
            arg++;
            continue;
        }
        if (std::string(argv[arg]) == "-v") {
            char* token = std::strtok(argv[arg + 1], ",");
            int i = 0;
            while (token != NULL) {
                eye[i++] = atof(token);
                token = std::strtok(NULL, ",");
            }

            if (i != 3) throw std::runtime_error("Error parsing view");
            hasEye = true;
            arg++;
            continue;
        }
        if (std::string(argv[arg]) == "-l") {
            char* token = std::strtok(argv[arg + 1], ",");
            int i = 0;

            while (token != NULL) {
                look[i++] = atof(token);
                token = std::strtok(NULL, ",");
            }

            if (i != 3) throw std::runtime_error("Error parsing view");
            hasLook = true;
            arg++;
            continue;
        }
        if (std::string(argv[arg]) == "-w") {
            char* token = std::strtok(argv[arg + 1], ",");
            int i = 0;

            while (token != NULL) {
                look[i++] = atof(token);
                token = std::strtok(NULL, ",");
            }

            if (i != 2) throw std::runtime_error("Error parsing window size");
            hasLook = true;
            arg++;
            continue;
        }
    }

    tracer::scene SceneMesh;
    bool ModelLoaded = false;

    if (modelname != "") {
        SceneMesh = model::loadobj(modelname);
        ModelLoaded = true;
    }

    int image_width = windowSize.x;
    int image_height = windowSize.y;

    tracer::camera cam(
        eye, look, tracer::vec3<float>(0, 1, 0), 60, float(image_width) / image_height);

    // Render
    auto image = std::vector<tracer::vec3<float>>(image_height * image_width);
    auto start_time = std::chrono::high_resolution_clock::now();

    RenderEnv render_env = {
        .image_width = image_width,
        .image_height = image_height,
        .image = image,
        .cam = cam,
        .SceneMesh = SceneMesh};

    // Create workers
    std::vector<std::thread> threads;
    {
        Sender tx;
        for (int i = 0; i < std::thread::hardware_concurrency(); i++) {
            threads.push_back(std::thread(worker, tx.make_receiver(), render_env));
        }

        // Populate work queue
        for (int line = 0; line < image_height; line++) {
            tx.send({line, line + 1});
        }
    }
    for (auto& t : threads) {
        t.join();
    }

    auto end_time = std::chrono::high_resolution_clock::now();

    std::cerr
        << "\n\n Duration : "
        << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()
        << std::endl;

    std::ofstream file(outputname, std::ios::out);

    file << "P3\n" << image_width << " " << image_height << "\n255\n";
    for (int h = image_height - 1; h >= 0; --h) {
        for (int w = 0; w < image_width; ++w) {
            auto& img = image[h * image_width + w];
            img.r = (img.r > 1.f) ? 1.f : img.r;
            img.g = (img.g > 1.f) ? 1.f : img.g;
            img.b = (img.b > 1.f) ? 1.f : img.b;

            file << int(img.r * 255) << " " << int(img.g * 255) << " " << int(img.b * 255) << "\n";
        }
    }
    return 0;
}
