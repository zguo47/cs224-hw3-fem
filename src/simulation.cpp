#include "simulation.h"
#include "graphics/meshloader.h"

#include <iostream>
#include<set>

using namespace Eigen;

Simulation::Simulation() {}

void Simulation::init()
{
    // STUDENTS: This code loads up the tetrahedral mesh in 'example-meshes/single-tet.mesh'
    //    (note: your working directory must be set to the root directory of the starter code
    //    repo for this file to load correctly). You'll probably want to instead have this code
    //    load up a tet mesh based on e.g. a file path specified with a command line argument.
    std::vector<Vector3d> vertices;
    std::vector<Vector4i> tets;
    if (MeshLoader::loadTetMesh(":/example-meshes/ellipsoid.mesh", vertices, tets)) {
        // STUDENTS: This code computes the surface mesh of the loaded tet mesh, i.e. the faces
        //    of tetrahedra which are on the exterior surface of the object. Right now, this is
        //    hard-coded for the single-tet mesh. You'll need to implement surface mesh extraction
        //    for arbitrary tet meshes. Think about how you can identify which tetrahedron faces
        //    are surface faces...
        std::vector<Vector3i> faces;
        double totalVolume = 0.0;
        for (const auto& tet : tets){
            std::vector<Vector3i> curr_faces;
            Vector3i firstface = Vector3i(tet[1], tet[0], tet[2]);
            Vector3i secondface = Vector3i(tet[2], tet[0], tet[3]);
            Vector3i thirdface = Vector3i(tet[3], tet[1], tet[2]);
            Vector3i fourthface = Vector3i(tet[3], tet[0], tet[1]);

            curr_faces.emplace_back(firstface);
            curr_faces.emplace_back(secondface);
            curr_faces.emplace_back(thirdface);
            curr_faces.emplace_back(fourthface);

            for (auto& f : curr_faces){

                if (std::find(faces.begin(), faces.end(), f) == faces.end()) {
                    faces.emplace_back(f);
                } else {
                    auto it = std::find(faces.begin(), faces.end(), f);
                    faces.erase(it);
                }
            }

            Vector3d a = vertices[tet[0]];
            Vector3d b = vertices[tet[1]];
            Vector3d c = vertices[tet[2]];
            Vector3d d = vertices[tet[3]];

            double volume = computeTetVolume(a, b, c, d);
            totalVolume += volume;

            TetrahedralElement* newtet = new TetrahedralElement();
            std::vector<int> nodes;
            nodes.push_back(tet[0]);
            nodes.push_back(tet[1]);
            nodes.push_back(tet[2]);
            nodes.push_back(tet[3]);
            newtet->nodes = nodes;
            m_tets.push_back(*newtet);

        }

        mass = rho * totalVolume;
        numtet = tets.size();

        for (Vector3d v : vertices){
            m_vertices.push_back(v);
        }

        ini_vertices = m_vertices;
        m_shape.init(vertices, faces, tets);
    }
    m_shape.setModelMatrix(Affine3f(Eigen::Translation3f(0, 3, 0)));


    double vertexm = mass / m_vertices.size();
    for (int i = 0; i < m_vertices.size(); i++){
        Vector3d g = 2.0 * Vector3d(0.0, -1.0, 0.0);
        vertexMass[i] = vertexm;
        Vector3d gravityForce = g * vertexMass[i];
        forces[i] = gravityForce;
        velocity[i] = Vector3d(0.0, 0.0, 0.0);
    }

    for (int i = 0; i < m_tets.size(); i++){
        std::vector<Vector3d> normals;
        std::vector<double> faceAreas;
        const Vector3d& v0 = m_vertices[m_tets[i].nodes[0]];
        const Vector3d& v1 = m_vertices[m_tets[i].nodes[1]];
        const Vector3d& v2 = m_vertices[m_tets[i].nodes[2]];
        const Vector3d& v3 = m_vertices[m_tets[i].nodes[3]];

        normals.push_back(computeNormal(v3, v1, v2));
        normals.push_back(computeNormal(v2, v0, v3));
        normals.push_back(computeNormal(v3, v0, v1));
        normals.push_back(computeNormal(v1, v0, v2));

        double face0 = computeFaceArea(v3, v1, v2);
        double face1 = computeFaceArea(v2, v0, v3);
        double face2 = computeFaceArea(v3, v0, v1);
        double face3 = computeFaceArea(v1, v0, v2);
        faceAreas.push_back(face0);
        faceAreas.push_back(face1);
        faceAreas.push_back(face2);
        faceAreas.push_back(face3);

        nodeNormals[i] = normals;
        nodeFaceAreas[i] = faceAreas;
    }

    std::vector<Vector3d> vertices2;
    std::vector<Vector4i> tets2;
    if (MeshLoader::loadTetMesh(":/example-meshes/sphere.mesh", vertices2, tets2)) {
        std::vector<Vector3i> faces;
        for (const auto& tet : tets2){
            std::vector<Vector3i> curr_faces;
            Vector3i firstface = Vector3i(tet[1], tet[0], tet[2]);
            Vector3i secondface = Vector3i(tet[2], tet[0], tet[3]);
            Vector3i thirdface = Vector3i(tet[3], tet[1], tet[2]);
            Vector3i fourthface = Vector3i(tet[3], tet[0], tet[1]);

            curr_faces.emplace_back(firstface);
            curr_faces.emplace_back(secondface);
            curr_faces.emplace_back(thirdface);
            curr_faces.emplace_back(fourthface);

            for (auto& f : curr_faces){

                if (std::find(faces.begin(), faces.end(), f) == faces.end()) {
                    faces.emplace_back(f);
                } else {
                    auto it = std::find(faces.begin(), faces.end(), f);
                    faces.erase(it);
                }
            }

        }

        m_shape2.init(vertices2, faces, tets2);
    }
    m_shape2.setModelMatrix(Affine3f(Eigen::Translation3f(1.0, 0.5, 0)));

    initGround();
}

void Simulation::update(double seconds)
{
    // STUDENTS: This method should contain all the time-stepping logic for your simulation.
    //   Specifically, the code you write here should compute new, updated vertex positions for your
    //   simulation mesh, and it should then call m_shape.setVertices to update the display with those
    //   newly-updated vertices.

    // STUDENTS: As currently written, the program will just continually compute simulation timesteps as long
    //    as the program is running (see View::tick in view.cpp) . You might want to e.g. add a hotkey for pausing
    //    the simulation, and perhaps start the simulation out in a paused state.

    // Note that the "seconds" parameter represents the amount of time that has passed since
    // the last update

    //=================midpoint

    seconds = 0.02f;

    // // Compute forces at the current state
    // gravity();
    // computeInternalForces(seconds);

    // // Save the current state
    // std::vector<Vector3d> originalVertices = m_vertices;
    // std::map<int, Vector3d> originalVelocities = velocity;
    // std::map<int, Vector3d> originalForces = forces;

    // // Compute the midpoint state
    // for (size_t i = 0; i < m_vertices.size(); ++i) {
    //     Vector3d midpointAcceleration = forces[i] / vertexMass[i];
    //     Vector3d midpointVelocity = velocity[i] + midpointAcceleration * (seconds / 2.0);
    //     m_vertices[i] = originalVertices[i] + midpointVelocity * (seconds / 2.0);
    // }

    // // Compute forces at the midpoint state
    // gravity();
    // computeInternalForces(seconds / 2.0);

    // for (size_t i = 0; i < m_vertices.size(); ++i) {
    //     // Recalculate midpoint velocity using forces at the midpoint
    //     Vector3d midpointAcceleration = forces[i] / vertexMass[i];
    //     Vector3d midpointVelocity = originalVelocities[i] + midpointAcceleration * (seconds / 2.0);

    //     // Use midpoint velocity to update positions
    //     m_vertices[i] = originalVertices[i] + midpointVelocity * seconds;

    //     // Update velocity using acceleration at the midpoint
    //     velocity[i] = originalVelocities[i] + midpointAcceleration * seconds;
    // }

    // m_shape.setVertices(m_vertices);
    // handleCollisions();

    //=================RK4

    std::vector<Vector3d> originalVertices = m_vertices;
    std::map<int, Vector3d> originalVelocities = velocity;

    std::map<int, Vector3d> k1, k2, k3, k4;

    // Compute k1

    gravity();
    computeInternalForces(seconds);
    for (size_t i = 0; i < m_vertices.size(); ++i) {
        Vector3d acceleration = forces[i] / vertexMass[i];
        k1[i] = seconds * acceleration;
    }

    // Compute k2 at the midpoint using original velocities updated by k1
    for (size_t i = 0; i < m_vertices.size(); ++i) {
        velocity[i] = originalVelocities[i] + k1[i] / 2.0; // use k1 to update velocity
    }

    gravity();
    computeInternalForces(seconds / 2.0);
    for (size_t i = 0; i < m_vertices.size(); ++i) {
        Vector3d acceleration = forces[i] / vertexMass[i];
        k2[i] = seconds * acceleration;
    }

    // Compute k3 at the midpoint using original velocities updated by k2
    for (size_t i = 0; i < m_vertices.size(); ++i) {
        velocity[i] = originalVelocities[i] + k2[i] / 2.0; // use k2 to update velocity
    }

    gravity();
    computeInternalForces(seconds / 2.0);
    for (size_t i = 0; i < m_vertices.size(); ++i) {
        Vector3d acceleration = forces[i] / vertexMass[i];
        k3[i] = seconds * acceleration;
    }

    // Compute k4 at the endpoint using original velocities updated by k3
    for (size_t i = 0; i < m_vertices.size(); ++i) {
        velocity[i] = originalVelocities[i] + k3[i]; // use k3 to update velocity
    }

    gravity();
    computeInternalForces(seconds);
    for (size_t i = 0; i < m_vertices.size(); ++i) {
        Vector3d acceleration = forces[i] / vertexMass[i];
        k4[i] = seconds * acceleration;
    }

    // Combine k1, k2, k3, k4 to update the final velocities and positions
    for (size_t i = 0; i < m_vertices.size(); ++i) {
        velocity[i] = originalVelocities[i] + (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6.0;
        m_vertices[i] = originalVertices[i] + velocity[i] * seconds;
    }

    m_shape.setVertices(m_vertices);
    handleCollisions();
}

void Simulation::clearForces(){
    for (int i = 0; i < m_vertices.size(); i++){
        forces[i] = Vector3d(0.0, 0.0, 0.0);
    }
}

void Simulation::gravity(){
    Vector3d g = 1.0 * Vector3d(0.0, -1.0, 0.0);
    for (int i = 0; i < m_vertices.size(); i++){
        Vector3d gravityForce = g * vertexMass[i];
        forces[i] = gravityForce;
    }
}

double Simulation::computeTetVolume(const Vector3d& a, const Vector3d& b, const Vector3d& c, const Vector3d& d) {
    return std::abs((a - d).dot((b - d).cross(c - d))) / 6.0;
}

double Simulation::computeFaceArea(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c){
    Vector3d edge1 = b - a;
    Vector3d edge2 = c - a;

    Vector3d crossProduct = edge1.cross(edge2);

    double area = 0.5 * crossProduct.norm();
    return area;
}

Vector3d Simulation::computeNormal(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c){
    Vector3d edge1 = b - a;
    Vector3d edge2 = c - a;

    Vector3d crossProduct = edge1.cross(edge2);

    crossProduct.normalize();
    return crossProduct;
}

Eigen::Matrix3d Simulation::computeDeformationGradient(const Simulation::TetrahedralElement& element, const std::vector<Vector3d>& initialNodes, const std::vector<Vector3d>& currentNodes) {
    // Calculate beta
    Eigen::Matrix3d beta;
    for (int i = 0; i < 3; ++i) {
        beta.col(i) = initialNodes[element.nodes[i]] - initialNodes[element.nodes[3]];
    }

    // Calculate P
    Eigen::Matrix3d P;
    for (int i = 0; i < 3; ++i) {
        P.col(i) = currentNodes[element.nodes[i]] - currentNodes[element.nodes[3]];
    }

    // Deformation gradient F = P * beta.inverse()
    Eigen::Matrix3d F = P * beta.inverse();
    return F;
}

Eigen::Matrix3d Simulation::computeVelocityGradient(const Simulation::TetrahedralElement& element,
                                                    const std::vector<Vector3d>& initialNodes,
                                                    const std::vector<Vector3d>& currentNodes,
                                                    std::map<int, Eigen::Vector3d>& currentVelocities) {

    // Calculate beta (same as in computeDeformationGradient)
    Eigen::Matrix3d beta;
    for (int i = 0; i < 3; ++i) {
        beta.col(i) = initialNodes[element.nodes[i]] - initialNodes[element.nodes[3]];
    }

    // Calculate V (Velocity matrix in world space)
    Eigen::Matrix3d V;
    for (int i = 0; i < 3; ++i) {
        V.col(i) = currentVelocities[element.nodes[i]] - currentVelocities[element.nodes[3]];
    }

    // Velocity gradient nabla_v = V * beta.inverse()
    Eigen::Matrix3d nabla_v = V * beta.inverse();
    return nabla_v;
}

Eigen::Matrix3d Simulation::computeGreensStrain(const Eigen::Matrix3d& F) {
    // Compute Green's strain
    Eigen::Matrix3d E = F.transpose() * F - Eigen::Matrix3d::Identity();
    return E;
}

Eigen::Matrix3d Simulation::computeStress(const Eigen::Matrix3d& strainRate, const Eigen::Matrix3d& strain, double rigidity, double incompress, double psi, double phi) {

    double lambda = incompress;
    double mu = rigidity;

    // Compute the stress using Hooke's law for linear elasticity
    Eigen::Matrix3d stress1 = lambda * strain.trace() * Eigen::Matrix3d::Identity() + 2 * mu * strain;

    Eigen::Matrix3d stress2 = phi * Eigen::Matrix3d::Identity() * strainRate.trace() + 2 * psi * strainRate;
    return stress1 + stress2;
}

Eigen::VectorXd Simulation::computeElementForceVector(int idx, Eigen::Matrix3d F, const Eigen::Matrix3d& stress, const Simulation::TetrahedralElement& element, const std::vector<Vector3d>& nodes) {

    std::vector<Vector3d> currentnodes = nodeNormals[idx];
    std::vector<double> currentNodeAreas = nodeFaceAreas[idx];

    // Compute the element force vector
    Eigen::VectorXd elementForceVector(12); // 12 entries for 4 nodes, each with 3 components (x, y, z)
    for (int i = 0; i < 4; ++i) { // Loop over each node
        Eigen::Vector3d force = F * stress * currentNodeAreas[i] * currentnodes[i]; // Uniformly distribute the force among the nodes
        elementForceVector.segment<3>(3 * i) = force;
    }

    return elementForceVector;
}

void Simulation::updateForcesMap(const Eigen::VectorXd& elementForceVector, const TetrahedralElement& element, std::map<int, Eigen::Vector3d>& forces) {
    for (int i = 0; i < 4; ++i) { // Loop over each node
        int nodeIndex = element.nodes[i];
        forces[nodeIndex] += elementForceVector.segment<3>(3 * i);
    }
}

void Simulation::computeInternalForces(double seconds){
    // Loop over each tetrahedral element
    int idx = 0;

    for (const auto& element : m_tets) {
        // Compute the deformation gradient for the element
        Eigen::Matrix3d F = computeDeformationGradient(element, ini_vertices, m_vertices);

        Eigen::Matrix3d vGradient = computeVelocityGradient(element, ini_vertices, m_vertices, velocity);

        // Compute Green's strain for the element
        Eigen::Matrix3d E = computeGreensStrain(F);

        Matrix3d strainRate = F.transpose() * vGradient + vGradient.transpose() * F;

        // Compute the stress for the element
        Eigen::Matrix3d stress = computeStress(strainRate, E, rigidity, incompress, psi, phi);

        // Compute the element force vector
        Eigen::VectorXd elementForceVector = computeElementForceVector(idx, F, stress, element, m_vertices);

        // Update the forces map with the element forces
        updateForcesMap(elementForceVector, element, forces);

        idx += 1;
    }
}

void Simulation::handleCollisions(){
    Vector3d groundNormal(0, 1, 0); // Assuming the ground plane is horizontal
    double groundHeight = -3.0; // Assuming the ground plane is at y = 0
    Vector3d sphereCenter(1.0, -2.5, 0); // Center of the sphere
    double sphereRadius = 1.0;
    double restitutionCoefficient = 1.0; // Coefficient of restitution
    double frictionCoefficient = 0.5; // Coefficient of friction



    for (size_t i = 0; i < m_vertices.size(); ++i) {

        // Check if the vertex penetrates the ground plane
        if (m_vertices[i].y() < groundHeight && m_vertices[i].x() >= -5 && m_vertices[i].x() <= 5 && m_vertices[i].z() >= -5 && m_vertices[i].z() <= 5) {
            // Project the vertex out of the collider (ground plane)
            m_vertices[i].y() = groundHeight;

            // Decompose the velocity into normal and tangential components
            double normalVelocity = velocity[i].dot(groundNormal);
            Eigen::Vector3d normalComponent = normalVelocity * groundNormal;
            Eigen::Vector3d tangentialComponent = velocity[i] - normalComponent;

            // Reflect the normal component of the velocity and scale by restitution coefficient
            normalComponent = -restitutionCoefficient * normalComponent;

            // Scale the tangential component of the velocity by friction coefficient
            tangentialComponent = frictionCoefficient * tangentialComponent;

            // Update the velocity with the new components
            velocity[i] = normalComponent + tangentialComponent;
        }


        // Check if the vertex penetrates the sphere
        Vector3d toVertex = m_vertices[i] - sphereCenter;
        double restitutionCoefficient2 = 1.0; // Coefficient of restitution
        double frictionCoefficient2 = 0.5; // Coefficient of friction
        double distance = toVertex.norm();
        if (distance < sphereRadius) {
            // Project the vertex out of the collider (sphere)
            Vector3d sphereNormal = toVertex.normalized();
            m_vertices[i] = sphereCenter + sphereNormal * sphereRadius;

            // Decompose the velocity into normal and tangential components
            double normalVelocity = velocity[i].dot(sphereNormal);
            Vector3d normalComponent = normalVelocity * sphereNormal;
            Vector3d tangentialComponent = velocity[i] - normalComponent;

            // Reflect the normal component of the velocity and scale by restitution coefficient
            normalComponent = -restitutionCoefficient2 * normalComponent;

            // Scale the tangential component of the velocity by friction coefficient
            tangentialComponent = frictionCoefficient2 * tangentialComponent;

            // Update the velocity with the new components
            velocity[i] = normalComponent + tangentialComponent;
        }
    }
}


void Simulation::draw(Shader *shader)
{
    m_shape.draw(shader);
    m_shape2.draw(shader);
    m_ground.draw(shader);
}

void Simulation::toggleWire()
{
    m_shape.toggleWireframe();
    m_shape2.toggleWireframe();
}

void Simulation::initGround()
{
    std::vector<Vector3d> groundVerts;
    std::vector<Vector3i> groundFaces;
    groundVerts.emplace_back(-5, 0, -5);
    groundVerts.emplace_back(-5, 0, 5);
    groundVerts.emplace_back(5, 0, 5);
    groundVerts.emplace_back(5, 0, -5);
    groundFaces.emplace_back(0, 1, 2);
    groundFaces.emplace_back(0, 2, 3);
    m_ground.init(groundVerts, groundFaces);
}
