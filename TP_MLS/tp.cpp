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

#include "chrono.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "src/Camera.h"
#include "src/Vec3.h"
#include "src/jmkdtree.h"
#include <GL/glut.h>
#include <algorithm>
#include <array>
#include <float.h>
#include <random>
#include <set>
#include <map>

std::vector<Vec3> positions;
std::vector<Vec3> normals;

std::vector<Vec3> positions2;
std::vector<Vec3> normals2;

std::vector<Vec3> g_outputPositions;
std::vector<unsigned int> g_outputTriangles;


std::vector<Vec3> g_outputPositions_base;
std::vector<unsigned int> g_outputTriangles_base;

/* std::vector<Vec3> dbgGridNodesPos;
std::vector<Vec3> dbgGridNodesNeg;
std::vector<Vec3> dbgCentersPos;
std::vector<Vec3> dbgCentersNeg; */

namespace KernelType {
	enum KernelType {
		UNIFORME,
		GAUSSIEN
	};
}

namespace Direction {
	enum To {
		RIGHT,
		BACK,
		BOTTOM,
	};
}

// -------------------------------------------
// OpenGL/GLUT application code.
// -------------------------------------------

static GLint window;
static unsigned int SCREENWIDTH = 640;
static unsigned int SCREENHEIGHT = 480;
static Camera camera;
static bool mouseRotatePressed = false;
static bool mouseMovePressed = false;
static bool mouseZoomPressed = false;
static int lastX = 0, lastY = 0, lastZoom = 0;
static bool fullScreen = false;

// ------------------------------------------------------------------------------------------------------------
// i/o and some stuff
// ------------------------------------------------------------------------------------------------------------
void loadPN(const std::string& filename, std::vector<Vec3>& o_positions, std::vector<Vec3>& o_normals) {
	unsigned int surfelSize = 6;
	FILE* in = fopen(filename.c_str(), "rb");
	if (in == NULL) {
		std::cout << filename << " is not a valid PN file." << std::endl;
		return;
	}
	size_t READ_BUFFER_SIZE = 1000; // for example...
	float* pn = new float[surfelSize * READ_BUFFER_SIZE];
	o_positions.clear();
	o_normals.clear();
	while (!feof(in)) {
		unsigned numOfPoints = fread(pn, 4, surfelSize * READ_BUFFER_SIZE, in);
		for (unsigned int i = 0; i < numOfPoints; i += surfelSize) {
			o_positions.push_back(Vec3(pn[i], pn[i + 1], pn[i + 2]));
			o_normals.push_back(Vec3(pn[i + 3], pn[i + 4], pn[i + 5]));
		}

		if (numOfPoints < surfelSize * READ_BUFFER_SIZE)
			break;
	}
	fclose(in);
	delete[] pn;
}
void savePN(const std::string& filename, std::vector<Vec3> const& o_positions, std::vector<Vec3> const& o_normals) {
	if (o_positions.size() != o_normals.size()) {
		std::cout << "The pointset you are trying to save does not contain the same number of points and normals." << std::endl;
		return;
	}
	FILE* outfile = fopen(filename.c_str(), "wb");
	if (outfile == NULL) {
		std::cout << filename << " is not a valid PN file." << std::endl;
		return;
	}
	for (unsigned int pIt = 0; pIt < o_positions.size(); ++pIt) {
		fwrite(&(o_positions[pIt]), sizeof(float), 3, outfile);
		fwrite(&(o_normals[pIt]), sizeof(float), 3, outfile);
	}
	fclose(outfile);
}
void scaleAndCenter(std::vector<Vec3>& io_positions) {
	Vec3 bboxMin(FLT_MAX, FLT_MAX, FLT_MAX);
	Vec3 bboxMax(FLT_MIN, FLT_MIN, FLT_MIN);
	for (unsigned int pIt = 0; pIt < io_positions.size(); ++pIt) {
		for (unsigned int coord = 0; coord < 3; ++coord) {
			bboxMin[coord] = std::min<float>(bboxMin[coord], io_positions[pIt][coord]);
			bboxMax[coord] = std::max<float>(bboxMax[coord], io_positions[pIt][coord]);
		}
	}
	Vec3 bboxCenter = (bboxMin + bboxMax) / 2.f;
	float bboxLongestAxis = std::max<float>(bboxMax[0] - bboxMin[0], std::max<float>(bboxMax[1] - bboxMin[1], bboxMax[2] - bboxMin[2]));
	for (unsigned int pIt = 0; pIt < io_positions.size(); ++pIt) {
		io_positions[pIt] = (io_positions[pIt] - bboxCenter) / bboxLongestAxis;
	}
}

void applyRandomRigidTransformation(std::vector<Vec3>& io_positions, std::vector<Vec3>& io_normals) {
	srand(time(NULL));
	Mat3 R = Mat3::RandRotation();
	Vec3 t = Vec3::Rand(1.f);
	for (unsigned int pIt = 0; pIt < io_positions.size(); ++pIt) {
		io_positions[pIt] = R * io_positions[pIt] + t;
		io_normals[pIt] = R * io_normals[pIt];
	}
}

void subsample(std::vector<Vec3>& i_positions, std::vector<Vec3>& i_normals, float minimumAmount = 0.1f, float maximumAmount = 0.2f) {
	std::vector<Vec3> newPos, newNormals;
	std::vector<unsigned int> indices(i_positions.size());
	for (unsigned int i = 0; i < indices.size(); ++i)
		indices[i] = i;
	srand(time(NULL));
	std::shuffle(indices.begin(), indices.end(), std::default_random_engine(std::random_device{}()));
	unsigned int newSize = indices.size() * (minimumAmount + (maximumAmount - minimumAmount) * (float)(rand()) / (float)(RAND_MAX));
	newPos.resize(newSize);
	newNormals.resize(newSize);
	for (unsigned int i = 0; i < newPos.size(); ++i) {
		newPos[i] = i_positions[indices[i]];
		newNormals[i] = i_normals[indices[i]];
	}
	i_positions = newPos;
	i_normals = newNormals;
}

bool save(const std::string& filename, std::vector<Vec3>& vertices, std::vector<unsigned int>& triangles) {
	std::ofstream myfile;
	myfile.open(filename.c_str());
	if (!myfile.is_open()) {
		std::cout << filename << " cannot be opened" << std::endl;
		return false;
	}

	myfile << "OFF" << std::endl;

	unsigned int n_vertices = vertices.size(), n_triangles = triangles.size() / 3;
	myfile << n_vertices << " " << n_triangles << " 0" << std::endl;

	for (unsigned int v = 0; v < n_vertices; ++v) {
		myfile << vertices[v][0] << " " << vertices[v][1] << " " << vertices[v][2] << std::endl;
	}
	for (unsigned int f = 0; f < n_triangles; ++f) {
		myfile << 3 << " " << triangles[3 * f] << " " << triangles[3 * f + 1] << " " << triangles[3 * f + 2];
		myfile << std::endl;
	}
	myfile.close();
	return true;
}

// ------------------------------------------------------------------------------------------------------------
// rendering.
// ------------------------------------------------------------------------------------------------------------

void initLight() {
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

void init() {
	camera.resize(SCREENWIDTH, SCREENHEIGHT);
	initLight();
	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);
	glDepthFunc(GL_LESS);
	glEnable(GL_DEPTH_TEST);
	glClearColor(0.2f, 0.2f, 0.3f, 1.0f);
	glEnable(GL_COLOR_MATERIAL);
}

void drawTriangleMesh(std::vector<Vec3> const& i_positions, std::vector<unsigned int> const& i_triangles) {
	glBegin(GL_TRIANGLES);
	for (unsigned int tIt = 0; tIt < i_triangles.size() / 3; ++tIt) {
		unsigned int i0 = i_triangles[3 * tIt];
		unsigned int i1 = i_triangles[3 * tIt + 1];
		unsigned int i2 = i_triangles[3 * tIt + 2];
		const Vec3& p0 = i_positions[i0];
		const Vec3& p1 = i_positions[i1];
		const Vec3& p2 = i_positions[i2];
		Vec3 n = Vec3::cross(p1 - p0, p2 - p0);
		n.normalize();
		glNormal3f(n[0], n[1], n[2]);
		glVertex3f(p0[0], p0[1], p0[2]);
		glVertex3f(p1[0], p1[1], p1[2]);
		glVertex3f(p2[0], p2[1], p2[2]);
	}
	glEnd();
}

static void drawPoints(const std::vector<Vec3>& pts, const Vec3& color, float size) {
	if (pts.empty())
		return;
	glPointSize(size);
	glColor3f(color[0], color[1], color[2]);
	glBegin(GL_POINTS);
	for (const auto& p : pts) {
		glVertex3f(p[0], p[1], p[2]);
	}
	glEnd();
}

void draw() {
	glPointSize(2); // for example...

	glColor3f(0.8, 0.8, 1);
	// drawPointSet(positions , normals);

	glColor3f(1, 0.5, 0.5);
	// drawPointSet(positions2 , normals2);

	// drawPoints(dbgGridNodesPos, Vec3(0.6f, 0.6f, 0.6f), 2.0f);
	// drawPoints(dbgGridNodesNeg, Vec3(0.f, 0.6f, 0.6f), 2.0f);

	// drawPoints(dbgCentersPos, Vec3(0.1f, 0.9f, 0.1f), 2.0f);
	// drawPoints(dbgCentersNeg, Vec3(0.9f, 0.1f, 0.1f), 2.0f);

	drawTriangleMesh(g_outputPositions, g_outputTriangles);
	drawTriangleMesh(g_outputPositions_base, g_outputTriangles_base);
}

void display() {
	glLoadIdentity();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	camera.apply();
	draw();
	glFlush();
	glutSwapBuffers();
}

void idle() {
	glutPostRedisplay();
}

void key(unsigned char keyPressed, int x, int y) {
	switch (keyPressed) {
	case 'f':
		if (fullScreen == true) {
			glutReshapeWindow(SCREENWIDTH, SCREENHEIGHT);
			fullScreen = false;
		} else {
			glutFullScreen();
			fullScreen = true;
		}
		break;

	case 'w':
		GLint polygonMode[2];
		glGetIntegerv(GL_POLYGON_MODE, polygonMode);
		if (polygonMode[0] != GL_FILL)
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		else
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		break;

	default:
		break;
	}
	idle();
}

void mouse(int button, int state, int x, int y) {
	if (state == GLUT_UP) {
		mouseMovePressed = false;
		mouseRotatePressed = false;
		mouseZoomPressed = false;
	} else {
		if (button == GLUT_LEFT_BUTTON) {
			camera.beginRotate(x, y);
			mouseMovePressed = false;
			mouseRotatePressed = true;
			mouseZoomPressed = false;
		} else if (button == GLUT_RIGHT_BUTTON) {
			lastX = x;
			lastY = y;
			mouseMovePressed = true;
			mouseRotatePressed = false;
			mouseZoomPressed = false;
		} else if (button == GLUT_MIDDLE_BUTTON) {
			if (mouseZoomPressed == false) {
				lastZoom = y;
				mouseMovePressed = false;
				mouseRotatePressed = false;
				mouseZoomPressed = true;
			}
		}
	}
	idle();
}

void motion(int x, int y) {
	if (mouseRotatePressed == true) {
		camera.rotate(x, y);
	} else if (mouseMovePressed == true) {
		camera.move((x - lastX) / static_cast<float>(SCREENWIDTH), (lastY - y) / static_cast<float>(SCREENHEIGHT), 0.0);
		lastX = x;
		lastY = y;
	} else if (mouseZoomPressed == true) {
		camera.zoom(float(y - lastZoom) / SCREENHEIGHT);
		lastZoom = y;
	}
}

void reshape(int w, int h) {
	camera.resize(w, h);
}

//
/*
nuages pts
chaque pts voisinage avec kdtree

chaque voisin calc plan projeter sur le plan
faire ça x fois

a la fin on a un centroid

*/

void HPSS(const Vec3& inputPoint, Vec3& outputPoint, Vec3& outputNormal,
	  const std::vector<Vec3>& positions, const std::vector<Vec3>& normals, const BasicANNkdTree& kdTree,
	  int kernelType, float radius, size_t nbIterations = 10, unsigned int knn = 20) {

	const float r2 = radius * radius;
	Vec3 point{inputPoint};
	Vec3 normalOut(0.0f, 0.0f, 0.0f);
	// repeter nbIterations fois
	ANNidxArray id_nearest_neighbors = new ANNidx[knn];
	ANNdistArray square_distances_to_neighbors = new ANNdist[knn];
	for (size_t i{0}; i < nbIterations; ++i) {

		// trouver x voisins avec kdtree
		kdTree.knearest(point, knn, id_nearest_neighbors, square_distances_to_neighbors);
		Vec3 centroid(0.0f, 0.0f, 0.0f);
		Vec3 normal(0.0f, 0.0f, 0.0f);
		float sumWeight{0.0f};

		// calc plan en fonction des voisins
		for (unsigned int i{0}; i < knn; i++) {

			if (r2 < square_distances_to_neighbors[i])
				continue; // si c'est plus loins que notre radius on skip

			float weight{1.0f};
			if (kernelType == KernelType::GAUSSIEN) {
				weight = exp(-square_distances_to_neighbors[i] / r2);
			}

			// projection sur le plan du voisin
			Vec3 toProjected = point - positions[id_nearest_neighbors[i]];
			float projectionDistance = Vec3::dot(toProjected, normals[id_nearest_neighbors[i]]);
			Vec3 projectedPosition = point - projectionDistance * normals[id_nearest_neighbors[i]];

			sumWeight += weight;
			centroid += weight * projectedPosition;
			normal += weight * normals[id_nearest_neighbors[i]];
		}

		if (sumWeight > 0.0f) {
			centroid /= sumWeight;
			normal.normalize();
		}

		point = centroid;
		normalOut = normal;
	}
	delete[] id_nearest_neighbors;
	delete[] square_distances_to_neighbors;
	outputPoint = point;
	outputNormal = normalOut;
}

std::pair<Vec3, Vec3> boundingBox(const std::vector<Vec3>& positions) {
	if (positions.empty()) {
		return {Vec3(0.f, 0.f, 0.f), Vec3(0.f, 0.f, 0.f)};
	}

	Vec3 bboxMin(FLT_MAX, FLT_MAX, FLT_MAX);
	Vec3 bboxMax(-FLT_MAX, -FLT_MAX, -FLT_MAX);

	for (const Vec3& p : positions) {
		for (int c = 0; c < 3; ++c) {
			if (p[c] < bboxMin[c])
				bboxMin[c] = p[c];
			if (p[c] > bboxMax[c])
				bboxMax[c] = p[c];
		}
	}
	return {bboxMin, bboxMax};
}

float sdfHPSS(const Vec3& inputPoint, const std::vector<Vec3>& positions, const std::vector<Vec3>& normals, const BasicANNkdTree& kdTree,
	      int kernelType, float radius, size_t nbIterations = 10, unsigned int knn = 20) {

	Vec3 outputPoint;
	Vec3 outputNormal;

	// hpss avec le point de la grille
	HPSS(inputPoint, outputPoint, outputNormal, positions, normals, kdTree, kernelType, radius, nbIterations, knn);

	// vecteur entre le point de la grille et la point en output de hpss
	Vec3 toOutput = inputPoint - outputPoint;

	// calcul distance signé
	float signe = (Vec3::dot(toOutput, outputNormal) >= 0.f) ? 1.f : -1.f;
	float dist = toOutput.length();

	return signe * dist;
}

template <typename T>
using Array3D = std::vector<std::vector<std::vector<T>>>;

struct vecIndices {
	int indices[3];
	int& operator[](unsigned int c) {
		return indices[c];
	}
	int operator[](unsigned int c) const {
		return indices[c];
	}
};

void addTriangles(int i, int j, int k, Direction::To endEdge, const Array3D<Vec3>& Vertices, const Array3D<float>& sdfGrid,
                  const std::vector<Vec3>& positions, const std::vector<Vec3>& normals, const BasicANNkdTree& kdTree,
                  int kernelType, float radius, size_t nbIterations, unsigned int knn) {

    if (i == 0 && endEdge != Direction::RIGHT)
        return;
    if (j == 0 && endEdge != Direction::BOTTOM)
        return;
    if (k == 0 && endEdge != Direction::BACK)
        return;

    std::array<vecIndices, 4> indicesCubesToConnect;
    float sdfEdgeEnd{};
    switch (endEdge) {
    case Direction::RIGHT:
        indicesCubesToConnect[0] = {i, j - 1, k - 1};
        indicesCubesToConnect[1] = {i, j, k - 1};
        indicesCubesToConnect[2] = {i, j, k};
        indicesCubesToConnect[3] = {i, j - 1, k};
        sdfEdgeEnd = sdfGrid[i + 1][j][k];
        break;
    case Direction::BOTTOM:
        indicesCubesToConnect[0] = {i - 1, j, k - 1};
        indicesCubesToConnect[1] = {i - 1, j, k};
        indicesCubesToConnect[2] = {i, j, k};
        indicesCubesToConnect[3] = {i, j, k - 1};
        sdfEdgeEnd = sdfGrid[i][j + 1][k];
        break;
    case Direction::BACK:
        indicesCubesToConnect[0] = {i - 1, j - 1, k};
        indicesCubesToConnect[1] = {i, j - 1, k};
        indicesCubesToConnect[2] = {i, j, k};
        indicesCubesToConnect[3] = {i - 1, j, k};
        sdfEdgeEnd = sdfGrid[i][j][k + 1];
        break;
    }

    for (const vecIndices& indices : indicesCubesToConnect) {
        int iC{indices[0]};
        int jC{indices[1]};
        int kC{indices[2]};
        Vec3 centerHPSS;
        Vec3 normal;
        HPSS(Vertices[iC][jC][kC], centerHPSS, normal, positions, normals, kdTree, kernelType, radius, nbIterations, knn);
        g_outputPositions.push_back(centerHPSS);
    }

    int baseIndex = g_outputPositions.size() - 4;
    // change l'odre des triangles en fonction du changement de signe
    if (sdfGrid[i][j][k] >= sdfEdgeEnd) {
        g_outputTriangles.push_back(baseIndex);
        g_outputTriangles.push_back(baseIndex + 2);
        g_outputTriangles.push_back(baseIndex + 1);

        g_outputTriangles.push_back(baseIndex);
        g_outputTriangles.push_back(baseIndex + 3);
        g_outputTriangles.push_back(baseIndex + 2);
    } else {
        g_outputTriangles.push_back(baseIndex);
        g_outputTriangles.push_back(baseIndex + 1);
        g_outputTriangles.push_back(baseIndex + 2);

        g_outputTriangles.push_back(baseIndex);
        g_outputTriangles.push_back(baseIndex + 2);
        g_outputTriangles.push_back(baseIndex + 3);
    }
}

void dualContouring(const std::vector<Vec3>& positions, int gridSize,
		    const BasicANNkdTree& kdTree, int kernelType, float radius,
		    size_t nbIterations = 10, unsigned int knn = 20) {

	int cellGridSize{gridSize - 1};

	auto [min, max] = boundingBox(positions);

    Vec3 diag = max - min;
    float margin = 0.01f; 
    min -= margin * diag;
    max += margin * diag;

	// grille uniforme
	Array3D<float> sdfGrid(gridSize, std::vector<std::vector<float>>(gridSize, std::vector<float>(gridSize, 0.0f)));
	Array3D<Vec3> vec3Grid(cellGridSize, std::vector<std::vector<Vec3>>(cellGridSize, std::vector<Vec3>(cellGridSize, Vec3(0.0f, 0.0f, 0.0f))));

	Vec3 range{max - min};
	Vec3 step{range / std::max(1, gridSize - 1)};

	// calculer les sdf sur les sommets de la grille uniforme et calcul des points dans les cellules
	for (int i{0}; i < gridSize; i++) {
		for (int j{0}; j < gridSize; j++) {
			for (int k{0}; k < gridSize; k++) {
				Vec3 gridPosition(min[0] + i * step[0], min[1] + j * step[1], min[2] + k * step[2]);
				sdfGrid[i][j][k] = sdfHPSS(gridPosition, positions, normals, kdTree, kernelType, radius, nbIterations, knn);
				if (i < cellGridSize && j < cellGridSize && k < cellGridSize) {
					Vec3 center(
					    min[0] + (i + 0.5f) * step[0],
					    min[1] + (j + 0.5f) * step[1],
					    min[2] + (k + 0.5f) * step[2]);
					vec3Grid[i][j][k] = center;
				}
			}
		}
	}

	// parcourir les arretes pour connecter les points
	for (int i{0}; i < cellGridSize; i++) {
		for (int j{0}; j < cellGridSize; j++) {
			for (int k{0}; k < cellGridSize; k++) {
				// check changement signe sur les 3 arretes
				if ((sdfGrid[i][j][k] >= 0 && sdfGrid[i + 1][j][k] < 0) || (sdfGrid[i][j][k] < 0 && sdfGrid[i + 1][j][k] >= 0)) {
					addTriangles(i, j, k, Direction::To::RIGHT, vec3Grid, sdfGrid, positions, normals, kdTree, kernelType, radius, nbIterations, knn);
				}
				if ((sdfGrid[i][j][k] >= 0 && sdfGrid[i][j + 1][k] < 0) || (sdfGrid[i][j][k] < 0 && sdfGrid[i][j + 1][k] >= 0)) {
					addTriangles(i, j, k, Direction::To::BOTTOM, vec3Grid, sdfGrid, positions, normals, kdTree, kernelType, radius, nbIterations, knn);
				}
				if ((sdfGrid[i][j][k] >= 0 && sdfGrid[i][j][k + 1] < 0) || (sdfGrid[i][j][k] < 0 && sdfGrid[i][j][k + 1] >= 0)) {
					addTriangles(i, j, k, Direction::To::BACK, vec3Grid, sdfGrid, positions, normals, kdTree, kernelType, radius, nbIterations, knn);
				}
			}
		}
	}

	std::cout << "Nombre de triangles : " << g_outputTriangles.size() << '\n';
	std::cout << "Nombre de sommets : " << g_outputPositions.size() << '\n';
}


struct Polar {
    float r;
    float theta; 
    float phi; 
	int idx;
};

Polar cartToPolar(const Vec3& pRel) {
    Polar s;
    s.r = pRel.length();
    if (s.r == 0.f) {
        s.theta = 0.f;
        s.phi = 0.f;
        return s;
    }
    s.theta = std::atan2(pRel[1], pRel[0]);
    float zClamped = std::max(-1.f, std::min(1.f, pRel[2] / s.r));
    s.phi = std::acos(zClamped);
    return s;
}

struct Bin{
	int count{};
	float rMin{};
	float rMax{};
	float rMean{};
	std::vector<Polar> polars;

};

void embedWatermarkInBins(std::vector<Bin>& bins, const std::string& bits, float alpha){
    if(bits.empty() || bins.empty()) return;
    if(alpha <= 0.f || alpha >= 0.49f) return;

    size_t bCount = std::min(bits.size(), bins.size());

    const float targetHigh = 0.5f + alpha;
    const float targetLow  = 0.5f - alpha;

	size_t bitIdx = 0;
    for(size_t b = 0; b < bins.size() && bitIdx < bits.size(); ++b){
        Bin& bin = bins[b];
        if(bin.count == 0) continue;

        bool bit1 = (bits[bitIdx] == '1');

        int iter = 0;
        while(iter < 6){
			float mean{};
            if(bit1){
                if(bin.rMean >= targetHigh) break;
                float k = (1.f - 2.f*alpha)/(1.f + 2.f*alpha); 
                for(auto& p : bin.polars){
                    p.r = std::pow(std::clamp(p.r, 0.f, 1.f), k);
					mean += p.r;
				}


            }else{
                if(bin.rMean <= targetLow) break;
                float k = (1.f + 2.f*alpha)/(1.f - 2.f*alpha); 
                for(auto& p : bin.polars){
					p.r = std::pow(std::clamp(p.r, 0.f, 1.f), k);
					mean += p.r;
				}
            }
			bin.rMean = mean / bin.count;
            ++iter;
        }
		++bitIdx;
    }
}

void cho(const std::string& bits, float alpha){
	if(g_outputPositions.empty()) return;

	int nbBits {bits.size()};

	// calcul du barycentre
	Vec3 barycenter(0.0f, 0.0f, 0.0f);
	for(const Vec3& position : g_outputPositions){
		barycenter += position;
	}
	barycenter /= static_cast<float>(g_outputPositions.size());

	// trnasformation en coordonnées polaires
	float rMax{-INFINITY};
	std::vector<Polar> posPolar;
	posPolar.reserve(g_outputPositions.size());

	for(size_t i=0 ; i<g_outputPositions.size() ; ++i){
        Polar pol = cartToPolar(g_outputPositions[i] - barycenter);
        pol.idx = (int)i; // on stock l'id pour plus tard mettre a jour la position des sommets
        posPolar.push_back(pol);
        if(pol.r > rMax) rMax = pol.r; 
    }

	// histogramme
	std::vector<Bin> bins(static_cast<size_t>(std::ceil(rMax / alpha)), {0, INFINITY, -INFINITY, 0.0f});

	for(const Polar& pos : posPolar){
		int binIndex = (int)std::floor(pos.r / alpha);
		Bin& bin = bins[binIndex];
		bin.count++;
		if(pos.r < bin.rMin) bin.rMin = pos.r;
		if(pos.r > bin.rMax) bin.rMax = pos.r;
		bin.polars.push_back(pos);
	}

	// normalisation
	for(auto& bin : bins){
		float rM {};
		float denom {bin.rMax - bin.rMin};
		if(denom != 0){
			for(auto& pol : bin.polars){
				float rNorm = (pol.r - bin.rMin) / (bin.rMax - bin.rMin);
				pol.r = rNorm;
				rM += pol.r;
			}
			rM /= bin.count;
			bin.rMean = rM;
		}
	}

	embedWatermarkInBins(bins, bits, alpha);

	for(const auto& b : bins){
        if(b.count == 0) continue;
        float denom = b.rMax - b.rMin;
        for(const auto& pol : b.polars){
            if(pol.idx < 0) continue;
            float rFinal = (denom > 0.f) ? (b.rMin + pol.r * denom) : b.rMin;
            float sinPhi = std::sin(pol.phi);
            Vec3 dir(std::cos(pol.theta) * sinPhi,
                     std::sin(pol.theta) * sinPhi,
                     std::cos(pol.phi));
            g_outputPositions[(size_t)pol.idx] = barycenter + rFinal * dir;
        }
    }


}

std::string extractWatermark(size_t nbBits, float alpha){
	if(g_outputPositions.empty()) return "Positions vide";

	// calcul du barycentre
	Vec3 barycenter(0.0f, 0.0f, 0.0f);
	for(const Vec3& position : g_outputPositions){
		barycenter += position;
	}
	barycenter /= static_cast<float>(g_outputPositions.size());

	float rMax{-INFINITY};
	std::vector<Polar> posPolar;
	posPolar.reserve(g_outputPositions.size());
	for(size_t i=0 ; i<g_outputPositions.size() ; ++i){
        Polar pol = cartToPolar(g_outputPositions[i] - barycenter);
        pol.idx = (int)i;
        posPolar.push_back(pol);
        if(pol.r > rMax) rMax = pol.r;
    }

	// histogramme
	std::vector<Bin> bins(static_cast<size_t>(std::ceil(rMax / alpha)), {0, INFINITY, -INFINITY, 0.0f});

	for(const Polar& pos : posPolar){
		int binIndex = (int)std::floor(pos.r / alpha);
		Bin& bin = bins[binIndex];
		bin.count++;
		if(pos.r < bin.rMin) bin.rMin = pos.r;
		if(pos.r > bin.rMax) bin.rMax = pos.r;
		bin.polars.push_back(pos);
	}

	// normalisation
	for(auto& bin : bins){
		float rM {};
		float denom {bin.rMax - bin.rMin};
		if(denom != 0){
			for(auto& pol : bin.polars){
				float rNorm = (pol.r - bin.rMin) / (bin.rMax - bin.rMin);
				pol.r = rNorm;
				rM += pol.r;
			}
			rM /= bin.count;
			bin.rMean = rM;
		}
	}

    std::string out;
    out.reserve(nbBits);

    for(size_t b=0; b<bins.size() && out.size()<nbBits; ++b){
        if(bins[b].count == 0) continue;
        float m = bins[b].rMean;
        if(m > 0.5f ) out.push_back('1');
        else if(m < 0.5f ) out.push_back('0');
        else out.push_back('?');
    }

    return out;
}


static void translatePositions(std::vector<Vec3>& pts, const Vec3& t){
    for(auto& p : pts) p += t;
}

static std::pair<Vec3,Vec3> bbox(const std::vector<Vec3>& pts){
    if(pts.empty()) return {Vec3(0,0,0), Vec3(0,0,0)};
    Vec3 mn(FLT_MAX,FLT_MAX,FLT_MAX), mx(-FLT_MAX,-FLT_MAX,-FLT_MAX);
    for(const auto& p: pts){
        if(p[0]<mn[0]) mn[0]=p[0]; if(p[1]<mn[1]) mn[1]=p[1]; if(p[2]<mn[2]) mn[2]=p[2];
        if(p[0]>mx[0]) mx[0]=p[0]; if(p[1]>mx[1]) mx[1]=p[1]; if(p[2]>mx[2]) mx[2]=p[2];
    }
    return {mn,mx};
}

static void offsetWatermarkedMesh(float gapFactor = 1.2f){
    if(g_outputPositions.empty() || g_outputPositions_base.empty()) return;
    auto [mnB, mxB] = bbox(g_outputPositions_base);
    float dx = (mxB[0]-mnB[0]) * gapFactor;
    translatePositions(g_outputPositions, Vec3(dx, 0.f, 0.f));
}

double computeRMSE(const std::vector<Vec3>& base, const std::vector<Vec3>& water){
    if(base.size() != water.size() || base.empty()) return 0.0;
    double acc = 0.0;
    for(size_t i=0;i<base.size();++i){
        Vec3 d = water[i] - base[i];
        acc += d.length() * d.length();
    }
    return std::sqrt(acc / base.size());
}

int main(int argc, char** argv) {
	if (argc > 2) {
		exit(EXIT_FAILURE);
	}
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
	glutInitWindowSize(SCREENWIDTH, SCREENHEIGHT);
	window = glutCreateWindow("tp point processing");

	init();
	glutIdleFunc(idle);
	glutDisplayFunc(display);
	glutKeyboardFunc(key);
	glutReshapeFunc(reshape);
	glutMotionFunc(motion);
	glutMouseFunc(mouse);
	key('?', 0, 0);

	{
		// Load a first pointset, and build a kd-tree:
		loadPN("pointsets/igea.pn", positions, normals);

		BasicANNkdTree kdtree;
		kdtree.build(positions);

		Chrono timer;
		timer.start();
		dualContouring(positions, /*gridSize*/ 128, kdtree, KernelType::GAUSSIEN, /*radius*/ 1.f, /*nbIterations*/ 5, /*nb voisins*/ 10);
		timer.end();

		g_outputPositions_base = g_outputPositions;
		g_outputTriangles_base = g_outputTriangles;

		// Create a second pointset that is artificial, and project it on pointset1 using MLS techniques:
		positions2.resize(100000);
		normals2.resize(positions2.size());
		for (unsigned int pIt = 0; pIt < positions2.size(); ++pIt) {
			positions2[pIt] = Vec3(
			    -0.6 + 1.2 * (double)(rand()) / (double)(RAND_MAX),
			    -0.6 + 1.2 * (double)(rand()) / (double)(RAND_MAX),
			    -0.6 + 1.2 * (double)(rand()) / (double)(RAND_MAX));
			positions2[pIt].normalize();
			positions2[pIt] = 0.6 * positions2[pIt];
		}

		std::string bits = "11011";
		float aplha {0.01};
		cho(bits, aplha);
		std::string msgRecu = extractWatermark(bits.size(),  aplha);
		std::cout << "Message Envoyé : " << bits << ", Message Reçus : " << msgRecu << ", sont égaux ? : " << (bits==msgRecu) << '\n';
		std::cout << "RMSE : " << computeRMSE(g_outputPositions_base, g_outputPositions) << '\n';


		offsetWatermarkedMesh(1.1f);




		// PROJECT USING MLS (HPSS and APSS):
		// TODO

		/*
		std::vector<Vec3> projectdPositions(positions2.size());
		std::vector<Vec3> projectdNormals(positions2.size());

		for (size_t i = 0; i < positions2.size(); ++i) {
			HPSS(positions2[i], projectdPositions[i], projectdNormals[i], positions, normals, kdtree, KernelType::UNIFORME, 1.f, 10);
		}

		positions2 = projectdPositions;
		normals2 = projectdNormals;
		*/
	}

	glutMainLoop();
	return EXIT_SUCCESS;
}
