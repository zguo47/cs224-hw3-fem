#pragma once

#include "graphics/shape.h"
#include "map"

class Shader;

class Simulation
{
public:
    Simulation();

    struct TetrahedralElement {
        std::vector<int> nodes; // Indices of the four vertices (nodes) of the tetrahedron
    };

    void init();

    void update(double seconds);

    void gravity();

    void draw(Shader *shader);

    void toggleWire();

    double computeMass();

    double computeTetVolume(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c, const Eigen::Vector3d& d);

    double computeFaceArea(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c);

    Eigen::Vector3d computeNormal(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c);

    void computeShapeFunctions(const TetrahedralElement& element, const std::vector<Eigen::Vector3d>& vertices, Eigen::Vector3d point, double shapeFunctions[4]);

    Eigen::Matrix3d computeDeformationGradient(const TetrahedralElement& element, const std::vector<Eigen::Vector3d>& initialNodes, const std::vector<Eigen::Vector3d>& currentNodes);

    Eigen::Matrix3d computeVelocityGradient(const Simulation::TetrahedralElement& element,
                                                        const std::vector<Eigen::Vector3d>& initialNodes,
                                                        const std::vector<Eigen::Vector3d>& currentNodes,
                                                        std::map<int, Eigen::Vector3d>& currentVelocities);

    Eigen::Matrix3d computeGreensStrain(const Eigen::Matrix3d& F);

    Eigen::Matrix3d computeStress(const Eigen::Matrix3d& strainRate, const Eigen::Matrix3d& strain, double youngsModulus, double poissonRatio, double psi, double phi);

    Eigen::VectorXd computeElementForceVector(int idx, Eigen::Matrix3d F, const Eigen::Matrix3d& stress, const TetrahedralElement& element, const std::vector<Eigen::Vector3d>& nodes);

    void updateForcesMap(const Eigen::VectorXd& elementForceVector, const TetrahedralElement& element, std::map<int, Eigen::Vector3d>& forces);

    void computeInternalForces(double seconds);

    bool isPointInsideTet(const Eigen::Vector3d& point,
                          const Eigen::Vector3d& v0,
                          const Eigen::Vector3d& v1,
                          const Eigen::Vector3d& v2,
                          const Eigen::Vector3d& v3);

    void handleVertexTetrahedronCollision(size_t vertexIndex, const Eigen::Vector3d& v0, const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, const Eigen::Vector3d& v3, const Eigen::Vector3d& normal0, const Eigen::Vector3d& normal1, const Eigen::Vector3d& normal2, const Eigen::Vector3d& normal3, double restitutionCoefficient, double frictionCoefficient);

    void clearForces();

    void handleCollisions();

    std::vector<Eigen::Vector3d> ini_vertices;
    std::vector<Eigen::Vector3d> m_vertices;
    std::vector<TetrahedralElement> m_tets;
    std::map<int, Eigen::Vector3d> forces;
    std::map<int, Eigen::Vector3d> velocity;
    std::map<int, double> vertexMass;
    std::map<int, std::vector<Eigen::Vector3d>> nodeNormals;
    std::map<int, std::vector<double>> nodeFaceAreas;

    // parameters
    double rho = 800.0;
    double mass = 0.0;
    int numtet = 0;
    float rigidity = 4000;
    float incompress = 4000;
    float psi = 80;
    float phi = 100;

private:
    Shape m_shape;
    Shape m_shape2;

    Shape m_ground;
    void initGround();
};
