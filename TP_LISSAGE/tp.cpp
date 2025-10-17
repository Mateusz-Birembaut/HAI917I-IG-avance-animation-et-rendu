// -------------------------------------------
// gMini : a minimal OpenGL/GLUT application
// for 3D graphics.
// Copyright (C) 2006-2008 Tamy Boubekeur
// All rights reserved.
// -------------------------------------------

// -------------------------------------------
// Disclaimer: this code is dirty in the
// meaning that there is no attention paid to
// proper class attribute access, memory
// management or optimisation of any kind. It
// is designed for quick-and-dirty testing
// purpose.
// -------------------------------------------

#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <array>
#include <GL/glut.h>
#include <float.h>
#include "src/Vec3.h"
#include "src/Camera.h"

enum DisplayMode
{
    WIRE = 0,
    SOLID = 1,
    LIGHTED_WIRE = 2,
    LIGHTED = 3
};

struct Triangle
{
    inline Triangle()
    {
        v[0] = v[1] = v[2] = 0;
    }
    inline Triangle(const Triangle &t)
    {
        v[0] = t.v[0];
        v[1] = t.v[1];
        v[2] = t.v[2];
    }
    inline Triangle(unsigned int v0, unsigned int v1, unsigned int v2)
    {
        v[0] = v0;
        v[1] = v1;
        v[2] = v2;
    }
    unsigned int &operator[](unsigned int iv) { return v[iv]; }
    unsigned int operator[](unsigned int iv) const { return v[iv]; }
    inline virtual ~Triangle() {}
    inline Triangle &operator=(const Triangle &t)
    {
        v[0] = t.v[0];
        v[1] = t.v[1];
        v[2] = t.v[2];
        return (*this);
    }
    // membres indices des sommets du triangle:
    unsigned int v[3];
    float quality{};
};

struct Mesh
{
    std::vector<Vec3> vertices;         // array of mesh vertices positions
    std::vector<Vec3> normals;          // array of vertices normals useful for the display
    std::vector<Triangle> triangles;    // array of mesh triangles
    std::vector<Vec3> triangle_normals; // triangle normals to display face normals

    // Compute face normals for the display
    void computeTrianglesNormals()
    {

        // A faire : implémenter le calcul des normales par face
        // Attention commencer la fonction par triangle_normals.clear();
        // Iterer sur les triangles

        // La normal du triangle i est le resultat du produit vectoriel de deux ses arêtes e_10 et e_20 normalisé (e_10^e_20)
        // L'arete e_10 est représentée par le vecteur partant du sommet 0 (triangles[i][0]) au sommet 1 (triangles[i][1])
        // L'arete e_20 est représentée par le vecteur partant du sommet 0 (triangles[i][0]) au sommet 2 (triangles[i][2])

        // Normaliser et ajouter dans triangle_normales

        triangle_normals.clear();
        for (unsigned int i = 0; i < triangles.size(); i++)
        {
            const Vec3 &e0 = vertices[triangles[i][1]] - vertices[triangles[i][0]];
            const Vec3 &e1 = vertices[triangles[i][2]] - vertices[triangles[i][0]];
            Vec3 n = Vec3::cross(e0, e1);
            n.normalize();
            triangle_normals.push_back(n);
        }
    }

    void offset(Vec3 offset)
    {
        for (unsigned int i = 0; i < vertices.size(); i++)
            vertices[i] += offset;
    }

    // Compute vertices normals as the average of its incident faces normals
    void computeVerticesNormals()
    {
        // Utiliser weight_type : 0 uniforme, 1 aire des triangles, 2 angle du triangle

        // A faire : implémenter le calcul des normales par sommet comme la moyenne des normales des triangles incidents
        // Attention commencer la fonction par normals.clear();
        // Initializer le vecteur normals taille vertices.size() avec Vec3(0., 0., 0.)
        // Iterer sur les triangles

        // Pour chaque triangle i
        // Ajouter la normal au triangle à celle de chacun des sommets en utilisant des poids
        // 0 uniforme, 1 aire du triangle, 2 angle du triangle

        // Iterer sur les normales et les normaliser
        normals.clear();
        normals.resize(vertices.size(), Vec3(0., 0., 0.));
        for (unsigned int i = 0; i < triangles.size(); i++)
        {
            for (unsigned int t = 0; t < 3; t++)
                normals[triangles[i][t]] += triangle_normals[i];
        }
        for (unsigned int i = 0; i < vertices.size(); i++)
            normals[i].normalize();
    }

    void computeNormals()
    {
        computeTrianglesNormals();
        computeVerticesNormals();
    }
};

// Transformation made of a rotation and translation
struct Transformation
{
    Mat3 rotation;
    Vec3 translation;
};

bool contain(std::vector<unsigned int> const &i_vector, unsigned int element)
{
    for (unsigned int i = 0; i < i_vector.size(); i++)
    {
        if (i_vector[i] == element)
            return true;
    }
    return false;
}

void collect_one_ring(std::vector<Vec3> const &i_vertices,
                      std::vector<Triangle> const &i_triangles,
                      std::vector<std::vector<unsigned int>> &o_one_ring)
{
    o_one_ring.clear();
    o_one_ring.resize(i_vertices.size()); // one-ring of each vertex, i.e. a list of vertices with which it shares an edge
    // Parcourir les triangles et ajouter les voisins dans le 1-voisinage
    // Attention verifier que l'indice n'est pas deja present
    for (unsigned int i = 0; i < i_triangles.size(); i++)
    {
        // Tous les points opposés dans le triangle sont reliés
        for (int j = 0; j < 3; j++)
        {
            for (int k = 0; k < 3; k++)
            {
                if (j != k)
                {
                    if (!contain(o_one_ring[i_triangles[i][j]], i_triangles[i][k]))
                    {
                        o_one_ring[i_triangles[i][j]].push_back(i_triangles[i][k]);
                    }
                }
            }
        }
    }
}

// liste de voisins
// vector avec laplacien avant lissage point to barycentre des voisins + un pas (jouer avec nombre iterations)
// chaque sommets moyenne barycentre voisinage

// retourne le centroid d'un verteur de vertices
Vec3 centroid(const std::vector<Vec3> &vertices)
{
    Vec3 centroid(0.0f, 0.0f, 0.0f);
    for (const auto &v : vertices)
    {
        centroid += v;
    }
    return centroid / vertices.size();
}

// 1.B, Calcule le centroid des voisins puis on a le vecteur pour déplacer notre sommet en calculant le vecteur centroid - sommet courrant 
void computeLaplaceMean(Mesh &mesh, std::vector<Vec3> &Lu)
{
    std::vector<std::vector<unsigned int>> oneRing;
    collect_one_ring(mesh.vertices, mesh.triangles, oneRing);

    Lu.clear();
    Lu.resize(mesh.vertices.size());

    for (int i{0}; i < mesh.vertices.size(); ++i)
    {
        std::vector<Vec3> voisins;
        for (int j{0}; j < oneRing[i].size(); ++j)
        {
            voisins.push_back(mesh.vertices[oneRing[i][j]]);
        }
        Vec3 centroidV{centroid(voisins)};

        Lu[i] = (centroidV - mesh.vertices[i]);
    }
}

// 1.A, on réutilise la fonction pour calculer le vecteur de déplacement et la courbure sera égal a 0.5x la longueur du vecteur de déplacement
void calc_uniform_mean_curvature(Mesh &mesh, std::vector<float> &field)
{
    field.clear();
    field.resize(mesh.vertices.size(), 0.0f);

    std::vector<Vec3> Lu;
    computeLaplaceMean(mesh, Lu);

    for (int i{0}; i < mesh.vertices.size(); ++i)
    {
        field[i] = 0.5 * Lu[i].length();
    }
}

// 1.B, on calcule les vecteurs de déplacement de cahque sommets, on applique les déplacements, puis on recalcule les vecteurs ect. n fois
void uniformSmooth(Mesh &mesh, unsigned int _iters)
{
    std::vector<Vec3> laplacien;
    for (int i{0}; i < _iters; ++i)
    {
        computeLaplaceMean(mesh, laplacien);
        for (int j{0}; j < mesh.vertices.size(); ++j)
        {
            mesh.vertices[j] += 0.5 * laplacien[j];
        }
        mesh.computeNormals();
    }
}

// 1.D, comme smooth mais on multiplie par un facteur > 0 puis a la prochain itération < 0
void taubinSmooth(Mesh &mesh, unsigned int _iters, float a, float b)
{
    assert(a > 0);
    assert(b < 0);
    std::vector<Vec3> laplacien;
    std::array<float, 2> mults{a, b};
    for (int i{0}; i < _iters; ++i)
    {
        computeLaplaceMean(mesh, laplacien);
        for (int j{0}; j < mesh.vertices.size(); ++j)
        {
            mesh.vertices[j] += mults[i % 2] * laplacien[j];
        }
        mesh.computeNormals();
    }
}

// 2.0, on calcule la qualité en fonction du rapport entre le rayon du cercle circonscris et le plus petit coté.
// ensuite on normalise avec des percentile sinon le résultat etait visuellement pas incroyable
void calcTriangleQuality(Mesh &mesh)
{
    for (int i{0}; i < mesh.triangles.size(); ++i)
    {
        unsigned int v0 = mesh.triangles[i].v[0];
        unsigned int v1 = mesh.triangles[i].v[1];
        unsigned int v2 = mesh.triangles[i].v[2];

        Vec3 e1{mesh.vertices[v2] - mesh.vertices[v0]};
        Vec3 e2{mesh.vertices[v1] - mesh.vertices[v0]};
        Vec3 e3{mesh.vertices[v1] - mesh.vertices[v2]};

        float e1L = e1.length();
        float e2L = e2.length();
        float e3L = e3.length();

        float area = Vec3::cross(e1, e2).length() / 2.0f;

        float radius = (e1L * e2L * e3L) / (4 * area);
        mesh.triangles[i].quality = radius / (std::min(std::min(e1L, e2L), e3L));
    }

    std::vector<float> validQualities;
    for (const auto &tri : mesh.triangles)
    {
        if (tri.quality >= 0.0f)
        { 
            validQualities.push_back(tri.quality);
        }
    }

    std::sort(validQualities.begin(), validQualities.end());

    size_t idx5 = std::max(size_t(0), size_t(validQualities.size() * 0.05));
    size_t idx95 = std::min(validQualities.size() - 1, size_t(validQualities.size() * 0.95));

    float minQuality = validQualities[idx5];
    float maxQuality = validQualities[idx95];

    for (auto &tri : mesh.triangles)
    {
        if (tri.quality >= 0.0f)
        {
            float clamped = std::max(minQuality, std::min(maxQuality, tri.quality));
            tri.quality = (clamped - minQuality) / (maxQuality - minQuality);
        }
        else
        {
            tri.quality = 1.0f;
        }
    }
}

float cos_angle(const Vec3 &u, const Vec3 &v)
{
    float dotProduct = Vec3::dot(u, v);
    float normU = u.length();
    float normV = v.length();

    return dotProduct / (normU * normV);
}

float sin_angle(const Vec3 &u, const Vec3 &v)
{
    float crossProductL = Vec3::cross(u, v).length();
    float normU = u.length();
    float normV = v.length();

    return crossProductL / (normU * normV);
}

// on calcule pour chaque triangle les cos et sin pour avoir le cot d'un angle, on ajout cet angle a l'arrete opposé
// a la fin chaque arrete possède comme poids, la somme des deux angles cotangent opposé dans les triangles adjacents
void calcWeights(Mesh &mesh, const std::vector<std::vector<unsigned int>> &oneRing, std::vector<std::vector<float>> &weights)
{
    weights.clear();
    weights.resize(mesh.vertices.size());

    for (int i{0}; i < mesh.vertices.size(); ++i)
    {
        weights[i].resize(oneRing[i].size(), 0.0f);
    }

    for (int t{0}; t < mesh.triangles.size(); ++t)
    {
        unsigned int v0 = mesh.triangles[t].v[0];
        unsigned int v1 = mesh.triangles[t].v[1];
        unsigned int v2 = mesh.triangles[t].v[2];

        Vec3 p0 = mesh.vertices[v0];
        Vec3 p1 = mesh.vertices[v1];
        Vec3 p2 = mesh.vertices[v2];

        Vec3 e0 = p1 - p0;
        Vec3 e1 = p2 - p0;
        float cot0 = cos_angle(e0, e1) / sin_angle(e0, e1);

        // o ring de v1 chercher v2 donne le poids ici
        for (int i = 0; i < oneRing[v1].size(); ++i)
        {
            if (oneRing[v1][i] == v2)
            {
                weights[v1][i] += cot0;
                break;
            }
        }
        // faire l'inverse
        for (int i = 0; i < oneRing[v2].size(); ++i)
        {
            if (oneRing[v2][i] == v1)
            {
                weights[v2][i] += cot0;
                break;
            }
        }

        e0 = p2 - p1;
        e1 = p0 - p1;
        float cot1 = cos_angle(e0, e1) / sin_angle(e0, e1);

        for (int i = 0; i < oneRing[v0].size(); ++i)
        {
            if (oneRing[v0][i] == v2)
            {
                weights[v0][i] += cot1;
                break;
            }
        }

        for (int i = 0; i < oneRing[v2].size(); ++i)
        {
            if (oneRing[v2][i] == v0)
            {
                weights[v2][i] += cot1;
                break;
            }
        }

        e0 = p0 - p2;
        e1 = p1 - p2;
        float cot2 = cos_angle(e0, e1) / sin_angle(e0, e1);

        for (int i = 0; i < oneRing[v0].size(); ++i)
        {
            if (oneRing[v0][i] == v1)
            {
                weights[v0][i] += cot2;
                break;
            }
        }
        for (int i = 0; i < oneRing[v1].size(); ++i)
        {
            if (oneRing[v1][i] == v0)
            {
                weights[v1][i] += cot2;
                break;
            }
        }
    }
}


// pour chaque sommet, on fait la somme pondéré des vecteurs reliant le sommet à ses voisins,
// les poids sont calculés avec la fonction calcWeights
// a la fin on divise par la somme des poids pour normaliser
void computeLaplaceBeltrami(Mesh &mesh, std::vector<Vec3> &laplacienOut)
{
    std::vector<std::vector<unsigned int>> oneRing;
    collect_one_ring(mesh.vertices, mesh.triangles, oneRing);

    std::vector<std::vector<float>> weights;

    calcWeights(mesh, oneRing, weights);

    laplacienOut.clear();
    laplacienOut.resize(mesh.vertices.size(), Vec3(0.0f, 0.0f, 0.0f));

    for (int i = 0; i < mesh.vertices.size(); ++i)
    {
        Vec3 laplacien(0.0f, 0.0f, 0.0f);
        float weightSum = 0.0f;

        for (int j = 0; j < oneRing[i].size(); ++j)
        {
            unsigned int neighbor_idx = oneRing[i][j];
            float w = weights[i][j];

            Vec3 edge = mesh.vertices[neighbor_idx] - mesh.vertices[i];
            laplacien += w * edge;
            weightSum += w;
        }

        if (weightSum > 1e-6f)
        {
            laplacienOut[i] = laplacien / weightSum;
        }
    }
}


void calc_mean_curvature(Mesh &mesh, std::vector<float> &field)
{
    field.clear();
    field.resize(mesh.vertices.size(), 0.0f);

    std::vector<Vec3> laplacien;
    computeLaplaceBeltrami(mesh, laplacien);

    for (int i = 0; i < mesh.vertices.size(); ++i)
    {
        field[i] = 0.5f * laplacien[i].length();
    }
}

void beltramiSmooth(Mesh &mesh, unsigned int _iters)
{
    std::vector<Vec3> laplacien;
    for (int i{0}; i < _iters; ++i)
    {
        computeLaplaceBeltrami(mesh, laplacien);
        for (int j{0}; j < mesh.vertices.size(); ++j)
        {
            mesh.vertices[j] += 0.5 * laplacien[j];
        }
        mesh.computeNormals();
    }
}

// 3

// pour chaque triangle du maillage, on calcule l'angle au niveau de chaque sommet du triangle.
// on ajoute cet angle à la somme des angles déjà accumulés pour ce sommet (stockée dans le vecteur field)
// a la fin, chaque element du vecteur field contient la somme des angles des triangles auxquels le sommet appartient
// cette somme permet de calculer la courbure  au sommet avec (2*PI - somme des angles).
void calc_gauss_curvature(Mesh &mesh, std::vector<float> &field)
{
    field.clear();
    field.resize(mesh.vertices.size(), 0.0f);

    std::vector<Vec3> weights(mesh.vertices.size());

    for (int i{0}; i < mesh.triangles.size(); ++i)
    {

        const Triangle &t = mesh.triangles[i];

        const Vec3 &v0 = mesh.vertices[t.v[0]];
        const Vec3 &v1 = mesh.vertices[t.v[1]];
        const Vec3 &v2 = mesh.vertices[t.v[2]];

        field[t.v[0]] += acos(cos_angle(v1 - v0, v2 - v0));
        field[t.v[1]] += acos(cos_angle(v0 - v1, v2 - v1));
        field[t.v[2]] += acos(cos_angle(v0 - v2, v1 - v2));
    }

    for (auto &val : field)
    {
        val = 2 * M_PI - val;
    }
}

void addNoise(Mesh &mesh)
{
    const float factor{0.03};
    for (int i{0}; i < mesh.vertices.size(); ++i)
    {
        const Vec3 &p = mesh.vertices[i];
        const Vec3 &n = mesh.normals[i];

        mesh.vertices[i] = Vec3(p[0] + factor * ((double)(rand()) / (double)(RAND_MAX)) * n[0], p[1] + factor * ((double)(rand()) / (double)(RAND_MAX)) * n[1], p[2] + factor * ((double)(rand()) / (double)(RAND_MAX)) * n[2]);
    }
}

template <typename T>
void normalize(std::vector<T> &values)
{
    T min{std::numeric_limits<T>::max()};
    T max{std::numeric_limits<T>::lowest()};
    for (const auto &val : values)
    {
        if (val < min)
            min = val;
        else if (val > max)
            max = val;
    }

    for (auto &val : values)
    {
        val = (val - min) / (max - min);
    }
}

// Input mesh loaded at the launch of the application
Mesh mesh;
std::vector<float> current_field; // normalized filed of each vertex

Mesh mesh2;
Mesh mesh3;

bool display_normals;
bool display_smooth_normals;
bool display_mesh;
bool display_triangle_quality; // Bascule entre affichage qualité triangles vs current_field

DisplayMode displayMode;
int weight_type;

// -------------------------------------------
// OpenGL/GLUT application code.
// -------------------------------------------

static GLint window;
static unsigned int SCREENWIDTH = 1600;
static unsigned int SCREENHEIGHT = 900;
static Camera camera;
static bool mouseRotatePressed = false;
static bool mouseMovePressed = false;
static bool mouseZoomPressed = false;
static int lastX = 0, lastY = 0, lastZoom = 0;
static bool fullScreen = false;

// ------------------------------------
// File I/O
// ------------------------------------
bool saveOFF(const std::string &filename,
             std::vector<Vec3> const &i_vertices,
             std::vector<Vec3> const &i_normals,
             std::vector<Triangle> const &i_triangles,
             std::vector<Vec3> const &i_triangle_normals,
             bool save_normals = false)
{
    std::ofstream myfile;
    myfile.open(filename.c_str());
    if (!myfile.is_open())
    {
        std::cout << filename << " cannot be opened" << std::endl;
        return false;
    }

    myfile << "OFF" << std::endl;

    unsigned int n_vertices = i_vertices.size(), n_triangles = i_triangles.size();
    myfile << n_vertices << " " << n_triangles << " 0" << std::endl;

    for (unsigned int v = 0; v < n_vertices; ++v)
    {
        myfile << i_vertices[v][0] << " " << i_vertices[v][1] << " " << i_vertices[v][2] << " ";
        if (save_normals)
            myfile << i_normals[v][0] << " " << i_normals[v][1] << " " << i_normals[v][2] << std::endl;
        else
            myfile << std::endl;
    }
    for (unsigned int f = 0; f < n_triangles; ++f)
    {
        myfile << 3 << " " << i_triangles[f][0] << " " << i_triangles[f][1] << " " << i_triangles[f][2] << " ";
        if (save_normals)
            myfile << i_triangle_normals[f][0] << " " << i_triangle_normals[f][1] << " " << i_triangle_normals[f][2];
        myfile << std::endl;
    }
    myfile.close();
    return true;
}

void openOFF(std::string const &filename,
             std::vector<Vec3> &o_vertices,
             std::vector<Vec3> &o_normals,
             std::vector<Triangle> &o_triangles,
             std::vector<Vec3> &o_triangle_normals,
             bool load_normals = true)
{
    std::ifstream myfile;
    myfile.open(filename.c_str());
    if (!myfile.is_open())
    {
        std::cout << filename << " cannot be opened" << std::endl;
        return;
    }

    std::string magic_s;

    myfile >> magic_s;

    if (magic_s != "OFF")
    {
        std::cout << magic_s << " != OFF :   We handle ONLY *.off files." << std::endl;
        myfile.close();
        exit(1);
    }

    int n_vertices, n_faces, dummy_int;
    myfile >> n_vertices >> n_faces >> dummy_int;

    o_vertices.clear();
    o_normals.clear();

    for (int v = 0; v < n_vertices; ++v)
    {
        float x, y, z;

        myfile >> x >> y >> z;
        o_vertices.push_back(Vec3(x, y, z));

        if (load_normals)
        {
            myfile >> x >> y >> z;
            o_normals.push_back(Vec3(x, y, z));
        }
    }

    o_triangles.clear();
    o_triangle_normals.clear();
    for (int f = 0; f < n_faces; ++f)
    {
        int n_vertices_on_face;
        myfile >> n_vertices_on_face;

        if (n_vertices_on_face == 3)
        {
            unsigned int _v1, _v2, _v3;
            myfile >> _v1 >> _v2 >> _v3;

            o_triangles.push_back(Triangle(_v1, _v2, _v3));

            if (load_normals)
            {
                float x, y, z;
                myfile >> x >> y >> z;
                o_triangle_normals.push_back(Vec3(x, y, z));
            }
        }
        else if (n_vertices_on_face == 4)
        {
            unsigned int _v1, _v2, _v3, _v4;
            myfile >> _v1 >> _v2 >> _v3 >> _v4;

            o_triangles.push_back(Triangle(_v1, _v2, _v3));
            o_triangles.push_back(Triangle(_v1, _v3, _v4));
            if (load_normals)
            {
                float x, y, z;
                myfile >> x >> y >> z;
                o_triangle_normals.push_back(Vec3(x, y, z));
            }
        }
        else
        {
            std::cout << "We handle ONLY *.off files with 3 or 4 vertices per face" << std::endl;
            myfile.close();
            exit(1);
        }
    }
}

// ------------------------------------
// Application initialization
// ------------------------------------
void initLight()
{
    GLfloat light_position1[4] = {22.0f, 16.0f, 50.0f, 0.0f};
    GLfloat direction1[3] = {-52.0f, -16.0f, -50.0f};
    GLfloat color1[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    GLfloat ambient[4] = {0.3f, 0.3f, 0.3f, 0.5f};

    glLightfv(GL_LIGHT1, GL_POSITION, light_position1);
    glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, direction1);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, color1);
    glLightfv(GL_LIGHT1, GL_SPECULAR, color1);
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);
    glEnable(GL_LIGHT1);
    glEnable(GL_LIGHTING);
}

void init()
{
    camera.resize(SCREENWIDTH, SCREENHEIGHT);
    initLight();
    glCullFace(GL_BACK);
    glDisable(GL_CULL_FACE);
    glDepthFunc(GL_LESS);
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.2f, 0.2f, 0.3f, 1.0f);
    glEnable(GL_COLOR_MATERIAL);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

    display_normals = false;
    display_mesh = true;
    display_smooth_normals = true;
    displayMode = LIGHTED;
}

// ------------------------------------
// Rendering.
// ------------------------------------

void drawVector(Vec3 const &i_from, Vec3 const &i_to)
{

    glBegin(GL_LINES);
    glVertex3f(i_from[0], i_from[1], i_from[2]);
    glVertex3f(i_to[0], i_to[1], i_to[2]);
    glEnd();
}

void drawAxis(Vec3 const &i_origin, Vec3 const &i_direction)
{

    glLineWidth(4); // for example...
    drawVector(i_origin, i_origin + i_direction);
}

void drawReferenceFrame(Vec3 const &origin, Vec3 const &i, Vec3 const &j, Vec3 const &k)
{

    glDisable(GL_LIGHTING);
    glColor3f(0.8, 0.2, 0.2);
    drawAxis(origin, i);
    glColor3f(0.2, 0.8, 0.2);
    drawAxis(origin, j);
    glColor3f(0.2, 0.2, 0.8);
    drawAxis(origin, k);
    glEnable(GL_LIGHTING);
}

typedef struct
{
    float r; // ∈ [0, 1]
    float g; // ∈ [0, 1]
    float b; // ∈ [0, 1]
} RGB;

RGB scalarToRGB(float scalar_value) // Scalar_value ∈ [0, 1]
{
    RGB rgb;
    float H = scalar_value * 360., S = 1., V = 0.85,
          P, Q, T,
          fract;

    (H == 360.) ? (H = 0.) : (H /= 60.);
    fract = H - floor(H);

    P = V * (1. - S);
    Q = V * (1. - S * fract);
    T = V * (1. - S * (1. - fract));

    if (0. <= H && H < 1.)
        rgb = (RGB){.r = V, .g = T, .b = P};
    else if (1. <= H && H < 2.)
        rgb = (RGB){.r = Q, .g = V, .b = P};
    else if (2. <= H && H < 3.)
        rgb = (RGB){.r = P, .g = V, .b = T};
    else if (3. <= H && H < 4.)
        rgb = (RGB){.r = P, .g = Q, .b = V};
    else if (4. <= H && H < 5.)
        rgb = (RGB){.r = T, .g = P, .b = V};
    else if (5. <= H && H < 6.)
        rgb = (RGB){.r = V, .g = P, .b = Q};
    else
        rgb = (RGB){.r = 0., .g = 0., .b = 0.};

    return rgb;
}

void drawSmoothTriangleMesh(Mesh const &i_mesh, bool draw_field = false)
{
    glBegin(GL_TRIANGLES);
    for (unsigned int tIt = 0; tIt < i_mesh.triangles.size(); ++tIt)
    {
        if (draw_field && display_triangle_quality)
        {
            RGB color = scalarToRGB(i_mesh.triangles[tIt].quality);
            glColor3f(color.r, color.g, color.b);
        }

        for (unsigned int i = 0; i < 3; i++)
        {
            const Vec3 &p = i_mesh.vertices[i_mesh.triangles[tIt][i]]; // Vertex position
            const Vec3 &n = i_mesh.normals[i_mesh.triangles[tIt][i]];  // Vertex normal

            if (draw_field && !display_triangle_quality && current_field.size() > 0)
            {
                RGB color = scalarToRGB(current_field[i_mesh.triangles[tIt][i]]);
                glColor3f(color.r, color.g, color.b);
            }
            glNormal3f(n[0], n[1], n[2]);
            glVertex3f(p[0], p[1], p[2]);
        }
    }
    glEnd();
}

void drawTriangleMesh(Mesh const &i_mesh, bool draw_field = false)
{
    glBegin(GL_TRIANGLES);
    for (unsigned int tIt = 0; tIt < i_mesh.triangles.size(); ++tIt)
    {
        const Vec3 &n = i_mesh.triangle_normals[tIt]; 

        if (draw_field && display_triangle_quality)
        {
            RGB color = scalarToRGB(i_mesh.triangles[tIt].quality);
            glColor3f(color.r, color.g, color.b);
        }

        for (unsigned int i = 0; i < 3; i++)
        {
            const Vec3 &p = i_mesh.vertices[i_mesh.triangles[tIt][i]]; // Vertex position

            if (draw_field && !display_triangle_quality)
            {
                RGB color = scalarToRGB(current_field[i_mesh.triangles[tIt][i]]);
                glColor3f(color.r, color.g, color.b);
            }
            glNormal3f(n[0], n[1], n[2]);
            glVertex3f(p[0], p[1], p[2]);
        }
    }
    glEnd();
}

void drawMesh(Mesh const &i_mesh, bool draw_field = false)
{
    if (display_smooth_normals)
        drawSmoothTriangleMesh(i_mesh, draw_field); // Smooth display with vertices normals
    else
        drawTriangleMesh(i_mesh, draw_field); // Display with face normals
}

void drawVectorField(std::vector<Vec3> const &i_positions, std::vector<Vec3> const &i_directions)
{
    glLineWidth(1.);
    for (unsigned int pIt = 0; pIt < i_directions.size(); ++pIt)
    {
        Vec3 to = i_positions[pIt] + 0.02 * i_directions[pIt];
        drawVector(i_positions[pIt], to);
    }
}

void drawNormals(Mesh const &i_mesh)
{

    if (display_smooth_normals)
    {
        drawVectorField(i_mesh.vertices, i_mesh.normals);
    }
    else
    {
        std::vector<Vec3> triangle_baricenters;
        for (const Triangle &triangle : i_mesh.triangles)
        {
            Vec3 triangle_baricenter(0., 0., 0.);
            for (unsigned int i = 0; i < 3; i++)
                triangle_baricenter += i_mesh.vertices[triangle[i]];
            triangle_baricenter /= 3.;
            triangle_baricenters.push_back(triangle_baricenter);
        }

        drawVectorField(triangle_baricenters, i_mesh.triangle_normals);
    }
}

// Draw fonction
void draw()
{

    if (displayMode == LIGHTED || displayMode == LIGHTED_WIRE)
    {

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glEnable(GL_LIGHTING);
    }
    else if (displayMode == WIRE)
    {

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glDisable(GL_LIGHTING);
    }
    else if (displayMode == SOLID)
    {
        glDisable(GL_LIGHTING);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    glColor3f(0.8, 1, 0.8);
    drawMesh(mesh, true);
    drawMesh(mesh2, true);
    drawMesh(mesh3, true);

    if (displayMode == SOLID || displayMode == LIGHTED_WIRE)
    {
        glEnable(GL_POLYGON_OFFSET_LINE);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glLineWidth(1.0f);
        glPolygonOffset(-2.0, 1.0);

        glColor3f(0., 0., 0.);
        drawMesh(mesh, false);
        drawMesh(mesh2, false);
        drawMesh(mesh3, false);

        glDisable(GL_POLYGON_OFFSET_LINE);
        glEnable(GL_LIGHTING);
    }

    glDisable(GL_LIGHTING);
    if (display_normals)
    {
        glColor3f(1., 0., 0.);
        drawNormals(mesh);
    }

    glEnable(GL_LIGHTING);
}

void changeDisplayMode()
{
    if (displayMode == LIGHTED)
        displayMode = LIGHTED_WIRE;
    else if (displayMode == LIGHTED_WIRE)
        displayMode = SOLID;
    else if (displayMode == SOLID)
        displayMode = WIRE;
    else
        displayMode = LIGHTED;
}

void display()
{
    glLoadIdentity();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    camera.apply();
    draw();
    glFlush();
    glutSwapBuffers();
}

void idle()
{
    glutPostRedisplay();
}

// ------------------------------------
// User inputs
// ------------------------------------
// Keyboard event
void key(unsigned char keyPressed, int x, int y)
{
    switch (keyPressed)
    {
    case 'f':
        if (fullScreen == true)
        {
            glutReshapeWindow(SCREENWIDTH, SCREENHEIGHT);
            fullScreen = false;
        }
        else
        {
            glutFullScreen();
            fullScreen = true;
        }
        break;

    case 'w':
        changeDisplayMode();
        break;

    case 'n': // Press n key to display normals
        display_normals = !display_normals;
        break;

    case '1': // Toggle loaded mesh display
        display_mesh = !display_mesh;
        break;

    case 's': // Switches between face normals and vertices normals
        display_smooth_normals = !display_smooth_normals;
        break;

    case '+': // Changes weight type: 0 uniforme, 1 aire des triangles, 2 angle du triangle
        weight_type++;
        if (weight_type == 3)
            weight_type = 0;
        mesh.computeVerticesNormals(); // recalcul des normales avec le type de poids choisi
        break;

    case 't': // Toggle entre affichage qualité triangles et current_field
        display_triangle_quality = !display_triangle_quality;
        break;

    default:
        break;
    }
    idle();
}

// Mouse events
void mouse(int button, int state, int x, int y)
{
    if (state == GLUT_UP)
    {
        mouseMovePressed = false;
        mouseRotatePressed = false;
        mouseZoomPressed = false;
    }
    else
    {
        if (button == GLUT_LEFT_BUTTON)
        {
            camera.beginRotate(x, y);
            mouseMovePressed = false;
            mouseRotatePressed = true;
            mouseZoomPressed = false;
        }
        else if (button == GLUT_RIGHT_BUTTON)
        {
            lastX = x;
            lastY = y;
            mouseMovePressed = true;
            mouseRotatePressed = false;
            mouseZoomPressed = false;
        }
        else if (button == GLUT_MIDDLE_BUTTON)
        {
            if (mouseZoomPressed == false)
            {
                lastZoom = y;
                mouseMovePressed = false;
                mouseRotatePressed = false;
                mouseZoomPressed = true;
            }
        }
    }

    idle();
}

// Mouse motion, update camera
void motion(int x, int y)
{
    if (mouseRotatePressed == true)
    {
        camera.rotate(x, y);
    }
    else if (mouseMovePressed == true)
    {
        camera.move((x - lastX) / static_cast<float>(SCREENWIDTH), (lastY - y) / static_cast<float>(SCREENHEIGHT), 0.0);
        lastX = x;
        lastY = y;
    }
    else if (mouseZoomPressed == true)
    {
        camera.zoom(float(y - lastZoom) / SCREENHEIGHT);
        lastZoom = y;
    }
}

void reshape(int w, int h)
{
    camera.resize(w, h);
}

// ------------------------------------
// Start of graphical application
// ------------------------------------
int main(int argc, char **argv)
{
    if (argc > 2)
    {
        exit(EXIT_FAILURE);
    }
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
    glutInitWindowSize(SCREENWIDTH, SCREENHEIGHT);
    window = glutCreateWindow("TP HAI917I");

    init();
    glutIdleFunc(idle);
    glutDisplayFunc(display);
    glutKeyboardFunc(key);
    glutReshapeFunc(reshape);
    glutMotionFunc(motion);
    glutMouseFunc(mouse);
    key('?', 0, 0);

    display_triangle_quality = false; 

    // Mesh loaded with precomputed normals
    /*     openOFF("data/elephant_n.off", mesh.vertices, mesh.normals, mesh.triangles, mesh.triangle_normals);
        mesh.offset(Vec3(-1.5, 0.0, 0.0));
        mesh.computeNormals(); */

    openOFF("data/elephant_n.off", mesh2.vertices, mesh2.normals, mesh2.triangles, mesh2.triangle_normals);
    mesh2.computeNormals();

    /*     openOFF("data/elephant_n.off", mesh3.vertices, mesh3.normals, mesh3.triangles, mesh3.triangle_normals);
        mesh3.offset(Vec3(1.5, 0.0, 0.0)); // Décalage à droite
        mesh3.computeNormals();  */

    /*     addNoise(mesh); */
    //addNoise(mesh2);
    /*     addNoise(mesh3); */

    // uniformSmooth(mesh, 5);

    // uniformSmooth(mesh2, 10);

    // uniformSmooth(mesh3, 20);

    // taubinSmooth(mesh, 5, 0.3, -0.3);

    // taubinSmooth(mesh2, 10, 0.3, -0.3);

    // taubinSmooth(mesh3, 20, 0.3, -0.3);

    //uniformSmooth(mesh2, 10);

    //beltramiSmooth(mesh2, 10);
    calcTriangleQuality(mesh2);

    calc_uniform_mean_curvature(mesh2, current_field);
    normalize<float>(current_field);

    // A faire : normaliser les champs pour avoir une valeur flotante entre 0. et 1. dans current_field
    //***********************************************//

    glutMainLoop();
    return EXIT_SUCCESS;
}
