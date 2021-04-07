#include "QuadricDecimationMesh.h"

const QuadricDecimationMesh::VisualizationMode QuadricDecimationMesh::QuadricIsoSurfaces =
    NewVisualizationMode("Quadric Iso Surfaces");

void QuadricDecimationMesh::Initialize() {
    // Allocate memory for the quadric array
    size_t numVerts = mVerts.size();
    mQuadrics.reserve(numVerts);
    std::streamsize width = std::cerr.precision();  // store stream precision
    for (size_t i = 0; i < numVerts; i++) {

        // Compute quadric for vertex i here
        mQuadrics.push_back(createQuadricForVert(i));

        // Calculate initial error, should be numerically close to 0

        glm::vec3 v0 = mVerts[i].pos;
        glm::vec4 v(v0[0], v0[1], v0[2], 1);
        auto m = mQuadrics.back();

        // TODO CHECK
        auto error = glm::dot(v, (m * v));
        // std::cerr << std::scientific << std::setprecision(2) << error << " ";
    }
    std::cerr << std::setprecision(width) << std::fixed;  // reset stream precision

    // Run the initialize for the parent class to initialize the edge collapses
    DecimationMesh::Initialize();
}

/*! \lab2 Implement the computeCollapse here */
/*!
 * \param[in,out] collapse The edge collapse object to (re-)compute,
 * DecimationMesh::EdgeCollapse
 */
void QuadricDecimationMesh::computeCollapse(EdgeCollapse* collapse) {
    // Compute collapse->position and collapse->cost here
    // based on the quadrics at the edge endpoints

    glm::vec3 collapsePosition;
    float collapseCost;

    size_t v1 = e(collapse->halfEdge).vert;
    size_t v2 = e(e(collapse->halfEdge).pair).vert;

    glm::mat4 Q1 = mQuadrics[v1];
    glm::mat4 Q2 = mQuadrics[v2];

    //glm::mat4 Q1 = createQuadricForVert(e(collapse->halfEdge).vert);
    //glm::mat4 Q2 = createQuadricForVert(e(e(collapse->halfEdge).pair).vert);

    glm::mat4 Qbar = Q1 + Q2;
    glm::mat4 Qhat = Qbar;

    Qhat[0][3] = 0;
    Qhat[1][3] = 0;
    Qhat[2][3] = 0;
    Qhat[3][3] = 1;

    glm::vec4 zero = {0, 0, 0, 1};
    glm::vec4 vbar;
    

    if (glm::determinant(Qhat) < 0.000001f && glm::determinant(Qhat) > -0.000001f) {
        vbar = glm::inverse(Qhat) * zero;
     
        collapseCost = glm::dot(vbar,Qbar*vbar);
        collapsePosition = vbar;
        
    } 
    else {
        collapsePosition = (v(e(collapse->halfEdge).vert).pos + v(e(e(collapse->halfEdge).next).vert).pos) / 2.0f;
        collapseCost = glm::distance(collapsePosition, v(e(collapse->halfEdge).vert).pos);
    }

    collapse->position = collapsePosition;
    collapse->cost = collapseCost;
    
}

/*! After each edge collapse the vertex properties need to be updated */
void QuadricDecimationMesh::updateVertexProperties(size_t ind) {
    DecimationMesh::updateVertexProperties(ind);
    mQuadrics[ind] = createQuadricForVert(ind);
}

/*!
 * \param[in] indx vertex index, points into HalfEdgeMesh::mVerts
 */
glm::mat4 QuadricDecimationMesh::createQuadricForVert(size_t indx) const {
    glm::mat4 Q({0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f},
                {0.0f, 0.0f, 0.0f, 0.0f});

    // The quadric for a vertex is the sum of all the quadrics for the adjacent
    // faces Tip: Matrix4x4 has an operator +=

    std::vector faces = FindNeighborFaces(indx);
    
    for (int i = 0; i < faces.size(); i++) {
    
        Q += createQuadricForFace(faces[i]);
    
    }

    return Q;
}

/*!
 * \param[in] indx face index, points into HalfEdgeMesh::mFaces
 */
glm::mat4 QuadricDecimationMesh::createQuadricForFace(size_t indx) const {

    // Calculate the quadric (outer product of plane parameters) for a face
    // here using the formula from Garland and Heckbert

    float a = f(indx).normal.x;
    float b = f(indx).normal.y;
    float c = f(indx).normal.z;
    float d = -dot(v(e(f(indx).edge).vert).pos, f(indx).normal);

    glm::mat4 quadric;

    quadric[0][0] = a * a;
    quadric[1][0] = a * b;
    quadric[2][0] = a * c;
    quadric[3][0] = a * d;
    quadric[0][1] = a * b;
    quadric[1][1] = b * b;
    quadric[2][1] = b * c;
    quadric[3][1] = b * d;
    quadric[0][2] = a * c;
    quadric[1][2] = b * c;
    quadric[2][2] = c * c;
    quadric[3][2] = c * d;
    quadric[0][3] = a * d;
    quadric[1][3] = b * d;
    quadric[2][3] = c * d;
    quadric[3][3] = d * d;

    return quadric;
}

void QuadricDecimationMesh::Render() {
    DecimationMesh::Render();

    glEnable(GL_LIGHTING);
    glMatrixMode(GL_MODELVIEW);

    if (mVisualizationMode == QuadricIsoSurfaces) {
        // Apply transform
        glPushMatrix();  // Push modelview matrix onto stack

        // Implement the quadric visualization here
        std::cout << "Quadric visualization not implemented" << std::endl;

        // Restore modelview matrix
        glPopMatrix();
    }
}
