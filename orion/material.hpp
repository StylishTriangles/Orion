#ifndef ORION_MATERIAL_HPP
#define ORION_MATERIAL_HPP

#include <cmath>

#include <orion/math.hpp>
#include <orion/light.hpp>
#include <orion/texture.hpp>

namespace orion {

struct SolidSurface {
    vec3f color_ambient;
    vec3f color_diffuse;
    vec3f color_specular;
    vec3f color_emissive;
    float shininess, opacity;
};

class Material
{
public:
    enum Type {
        PHONG = 0,
    };
    enum TextureType {
        AMBIENT = 0,
        DIFFUSE = 1,
        SPECULAR = 2,
        EMISSIVE = 3,
        BUMP = 4,
        SIZE = 5,
    };
    Material() = default;
    explicit Material(const SolidSurface &sol) : sol(sol), type(PHONG) { texturesFromSolidColor(sol); }
    Material(const Material&) = default;
    ~Material() = default;

    // @brief Calculate color of surface
    // @param rayDir: direction of ray hitting this surface
    // @param normal: normal to the surface
    // @param hitPoint: point in 3D space at which surface was hit
    // @param light: light structure representing light
    vec3f color(const vec3f& rayDir, const vec3f& normal, const vec3f & hitPoint, const Light& light) const {
        if (type == PHONG) {
            // Ambient
            vec3f ambient = sol.color_ambient;

            // Diffuse
            vec3f norm = normalize(normal);
            vec3f lightDir = normalize(light.position - hitPoint);
            float diff = max(dot(norm, lightDir), 0.0f);
            vec3f diffuse = diff * sol.color_diffuse;

            // Specular
            vec3f viewDir = normalize(-rayDir);
            vec3f reflectDir = reflect(-lightDir, norm);
            float spec = 0.5f*std::pow(max(dot(viewDir, reflectDir), 0.0f), sol.shininess);
            vec3f specular = spec * sol.color_specular;

            return light.color * (ambient + diffuse + specular);
        }
        return vec3f(0.0f);
    }

    // @brief Calculate color of surface with textures
    // @param rayDir: direction of ray hitting this surface
    // @param normal: normal to the surface
    // @param hitPoint: point in 3D space at which surface was hit
    // @param light: light structure representing light
    // @param uv: uv mapping of the texture
    vec3f color(const vec3f& rayDir, const vec3f& normal, const vec3f & hitPoint, const Light& light, const vec2f& uv) const {
        if (type == PHONG) {
            // Use bump map if available
            vec3f norm = normal;
            if (!tex[BUMP].isSolidColor()) {
                norm = tex[BUMP].color(uv);
            }
            norm.normalize();
            // Ambient
            vec3f ambient = 0.0f;// tex[AMBIENT].color(uv);

            // Diffuse
            vec3f lightDir = normalize(light.position - hitPoint);
            float diff = max(dot(norm, lightDir), 0.0f);
            vec3f diffuse = diff * tex[DIFFUSE].color(uv);

            // Specular
            vec3f viewDir = normalize(-rayDir);
            vec3f reflectDir = reflect(-lightDir, norm);
            float spec = 0.5f*std::pow(max(dot(viewDir, reflectDir), 0.0f), sol.shininess);
            vec3f specular = spec * tex[SPECULAR].color(uv);

            return light.color * (ambient + diffuse + specular);
        }
        return vec3f(0.0f);
    }

    void texturesFromSolidColor(const SolidSurface& _sol) {
        setTexture(AMBIENT, _sol.color_ambient);
        setTexture(DIFFUSE, _sol.color_diffuse);
        setTexture(SPECULAR, _sol.color_specular);
        setTexture(EMISSIVE, _sol.color_emissive);
    }

    void setTexture(TextureType type, const Texture& texture) {
        tex[type] = texture;
    }

    Texture tex[TextureType::SIZE];
    SolidSurface sol;
    Type type;
};

}; // namespace orion

#endif // ORION_MATERIAL_HPP