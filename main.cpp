#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>

struct Vec3 {

    float x, y, z;
    Vec3(float x_ = 0, float y_ = 0, float z_ = 0) : x(x_), y(y_), z(z_) {}
    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(float s) const { return Vec3(x * s, y * s, z * s); }
    Vec3 operator*(const Vec3& v) const { return Vec3(x * v.x, y * v.y, z * v.z); } 
    float dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }
    Vec3 cross(const Vec3& v) const { return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x); }
    Vec3 normalize() const {
        float len = std::sqrt(dot(*this));
        return Vec3(x / len, y / len, z / len);
    }
    friend Vec3 operator*(float s, const Vec3& v) {
        return Vec3(v.x * s, v.y * s, v.z * s);
    }   

    Vec3 operator/(float s) const 
    {
        return Vec3(x / s, y / s, z / s);
    }

    float length()
    {
        return std::sqrt(dot(*this));
    }

    friend Vec3 operator-(float s, const Vec3& v) 
    {
        return Vec3(s - v.x, s - v.y, s - v.z);
    }
};

// Define a ray 
struct Ray {
    Vec3 origin, direction;
    Ray(Vec3 o, Vec3 d) : origin(o), direction(d.normalize()) {}
};

// Define a sphere object 
struct Sphere {

    Vec3 center;
    float radius;
    Vec3 color;
    Vec3 emissive;
    bool isEmissive;
    bool isReflective;
    Sphere(Vec3 c, float r, Vec3 col, Vec3 emissive, bool isEmissive = false, bool isReflective = false) : 
    center(c), radius(r), color(col), emissive(emissive), isEmissive{isEmissive}, isReflective{isReflective} {}
    
    bool intersect(const Ray& ray, float& t) const {
        Vec3 oc = ray.origin - center;
        float a = ray.direction.dot(ray.direction);
        float b = 2.0f * oc.dot(ray.direction);
        float c = oc.dot(oc) - radius * radius;
        float discriminant = b * b - 4 * a * c;
        if (discriminant < 0) return false;
        t = (-b - std::sqrt(discriminant)) / (2.0f * a);
        return t > 0;
    }
};

// Random number generator
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<float> dis(0.0f, 1.0f);

Vec3 cosrandomHemisphereDirection(const Vec3& normal) {

    float r1 = dis(gen); 
    float r2 = dis(gen); 

    float cosTheta = std::sqrt(1.0f - r1);
    float sinTheta = std::sqrt(r1);
    float phi = 2.0f * M_PI * r2;

    Vec3 localDir(sinTheta * std::cos(phi), cosTheta, sinTheta * std::sin(phi));

    Vec3 up = (std::abs(normal.y) < 0.999f) ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
    Vec3 tangent = up.cross(normal).normalize();
    Vec3 bitangent = normal.cross(tangent); // No need to normalize; it's already unit length

    return (tangent * localDir.x + bitangent * localDir.z + normal * localDir.y).normalize();
}

// check occlusion of a ray (used for shadow ray)
bool isOccluded(Ray shadowRay, const std::vector<Sphere>& scene, float maxDist) {
    for (size_t i = 0; i < scene.size(); ++i) {
        float tHit;
        if (scene[i].intersect(shadowRay, tHit) && tHit > 0 && tHit < maxDist) {
            return true; 
        }
    }
    return false;
}

// Not yet being used 
Vec3 FresnelSchlick(float cosTheta, Vec3 F0)
{
    return F0 + (Vec3(1.0f) - F0) * std::pow(std::clamp(1.0f - cosTheta, 0.0f, 1.0f), 5.0f);
}

// Path tracing 
Vec3 trace(Ray ray, const std::vector<Sphere>& scene, int depth, Vec3 lightPos) {
    
    if (depth <= 0) return Vec3(0, 0, 0);

    float t = 1e20;
    const Sphere* hitSphere = nullptr;
    for (size_t i = 0; i < scene.size(); ++i) {
        float tHit;
        if (scene[i].intersect(ray, tHit) && tHit < t) {
            t = tHit;
            hitSphere = &scene[i];
        }
    }

    if (!hitSphere) {
        Vec3 unit_direction = ray.direction.normalize();
         float a = 0.5 * (unit_direction.y + 1.0);
         return (1.0f - a) * Vec3(1.0, 1.0, 1.0) + (a * Vec3(0.5, 0.7, 1.0));
    }

    Vec3 hitPoint = ray.origin + ray.direction * t;
    Vec3 normal = (hitPoint - hitSphere->center).normalize();

    // check if it's reflective 
    if (hitSphere->isReflective) {
        Vec3 incidentDir = ray.direction.normalize(); // Ensure normalized incident direction
        Vec3 reflectDir = incidentDir - 2.0f * incidentDir.dot(normal) * normal;
        reflectDir = reflectDir.normalize(); // Ensure the reflected direction is normalized

        Ray reflectRay(hitPoint + normal * 0.001f, reflectDir);
        Vec3 reflectedColor = trace(reflectRay, scene, depth - 1, lightPos);
        return hitSphere->emissive + hitSphere->color * reflectedColor;;
    }

    Vec3 sunDirection = Vec3(0, 1.0f, 0);
    auto brdf = hitSphere->color / M_PI; // constant brdf for diffuse 
    Vec3 radiance = Vec3(0,0,0);
    float L_intensity = 1.0f;

    // shoot a shadow ray
    Ray directShadowRay(hitPoint + normal * 0.001f, sunDirection);
    float directCosTheta = std::max(0.0f, normal.dot(sunDirection)); 
    float visibility = isOccluded(directShadowRay, scene, 10000) == true ? 0.0f : 1.0f;
    radiance = radiance + (brdf * L_intensity * directCosTheta) * visibility;

    // this will now be indirect
    Vec3 omega_i = cosrandomHemisphereDirection(normal);
    Ray bounceRay(hitPoint + normal * 0.001f, omega_i);
    float cosTheta = std::max(0.0f, normal.dot(omega_i));
    Vec3 L_i = trace(bounceRay, scene, depth - 1, lightPos);

    float pdfValue = cosTheta / M_PI;

    if(pdfValue > 0.0f)
        radiance = radiance + (brdf * L_i * L_intensity * cosTheta) / pdfValue;


    if (!hitSphere->isEmissive) {

        Vec3 lightDir = (lightPos - hitPoint).normalize(); 
        float dist = (lightPos - hitPoint).length();       
        float NdotL = std::max(0.0f, normal.dot(lightDir));
        float spot_intensity = 1.0f;

        if (NdotL > 0.0f) { 
            Ray shadowRay(hitPoint + normal * 0.001f, lightDir);
            if (!isOccluded(shadowRay, scene, dist)) { 
                Vec3 spot_radiance = Vec3(0.5f, 0.3f, 0.0f) * spot_intensity / (dist * dist);
                radiance = radiance + (brdf * spot_radiance * NdotL);
            }
        }
    }
        
    return hitSphere->emissive + (radiance);
}

int main() {
    const int width = 1000;
    const int height = 1000;
    const int samples = 64;
    const int maxDepth = 4;

    std::vector<Sphere> scene;
    scene.push_back(Sphere(Vec3(0, -1001, -5), 1000, Vec3(0.8, 0.8, 0.8), Vec3(0, 0, 0))); // floor
    //scene.push_back(Sphere(Vec3(0, 1005, -5), 1000, Vec3(0.8, 0.8, 0.8), Vec3(1,1,1), true)); // ceiling
    //scene.push_back(Sphere(Vec3(0, 0, -1005), 1000, Vec3(0.8, 0.8, 0.8), Vec3(0, 0, 0), false, false)); // back wall
    //scene.push_back(Sphere(Vec3(-1004, 0, -5), 1000, Vec3(1.0, 0.2, 0.2), Vec3(0, 0, 0))); // left
    //scene.push_back(Sphere(Vec3(1004, 0, -5), 1000, Vec3(0.2, 1.0, 0.2), Vec3(0, 0, 0))); // right

    //scene.push_back(Sphere(Vec3(0, 6, 0), 2.0, Vec3(1,1,1), Vec3(1,1,1)));
    
    scene.push_back(Sphere(Vec3(-3, 0.0, -2), 1.0, Vec3(0.8, 0.8, 0.8), Vec3(0, 0, 0))); 
    scene.push_back(Sphere(Vec3(2, 0.0, -3), 1.0, Vec3(0.8, 0.8, 0.8), Vec3(0, 0, 0))); 
    scene.push_back(Sphere(Vec3(2, 0.0, 1), 1.0, Vec3(0.8, 0.8, 0.8), Vec3(0, 0, 0), false, true));

    // light pos 
    //scene.push_back(Sphere(Vec3(0, 2, -1), 0.0, Vec3(1, 0, 0), Vec3(15, 15, 15)));
    Vec3 lightPos(0, 3, -1); // Used for direct lighting if enabled

    // camera 
    Vec3 camera(0, 2, 8); // Move camera forward to see inside the box
    std::vector<Vec3> image(width * height);

    // Rendering loop 
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            Vec3 color(0, 0, 0);
            for (int s = 0; s < samples; ++s) {
                float u = (x + dis(gen)) / width - 0.5f;
                float v = (y + dis(gen)) / height - 0.5f;
                Vec3 dir(u, v, -1); 
                Ray ray(camera, dir);
                color = color + trace(ray, scene, maxDepth, lightPos);
            }
            // gamma correction 
            color = color * (1.0f / samples);
            color.x = pow(color.x, 1.0f / 2.2f);
            color.y = pow(color.y, 1.0f / 2.2f);
            color.z = pow(color.z, 1.0f / 2.2f);
            image[(height - 1 - y) * width + x] = color;
        }
    }

    // Output PPM 
    std::cout << "P3\n" << width << " " << height << "\n255\n";
    for (size_t i = 0; i < image.size(); ++i) {
        int r = std::min(255, std::max(0, int(image[i].x * 255)));
        int g = std::min(255, std::max(0, int(image[i].y * 255)));
        int b = std::min(255, std::max(0, int(image[i].z * 255)));
        std::cout << r << " " << g << " " << b << "\n";
    }

    return 0;
}