#include "polylidar/polylidar.hpp"


namespace polylidar
{

auto PL_NAN = std::numeric_limits<double>::quiet_NaN();

inline bool validateTriangle2D(size_t t, MeshHelper::HalfEdgeTriangulation &delaunay, Matrix<double> &points, Config &config)
{
    // auto maxXY = getMaxDimTriangle(t, delaunay, points);
    // std::cout << "Triangle " << t << " Radius: " << radius << std::endl;
    if (config.alpha > 0.0 && circumsribedRadius(t, delaunay, points) > config.alpha)
    {
        return false;
    }
    else if (config.xyThresh > 0.0 && getMaxDimTriangle(t, delaunay, points) > config.xyThresh)
    {
        return false;
    }
    else if (config.lmax > 0.0 && getMaxEdgeLength(t, delaunay, points) > config.lmax)
    {
        return false;
    }

    return true;
}

inline bool validateTriangle3D(size_t t, MeshHelper::HalfEdgeTriangulation &delaunay, Matrix<double> &points, Config &config)
{
    double zDiff = 0.0;
    std::array<double, 3> normal;
    // get zDiff and normal of triangle
    maxZChangeAndNormal(t, delaunay, points, zDiff, normal, config.desiredVector);
    // get dot product of triangle
    auto prod = std::abs(dotProduct3(normal, config.desiredVector));

    bool passZThresh = config.zThresh > 0.0 && zDiff < config.zThresh;

    return prod > config.normThresh || (passZThresh && prod > config.normThreshMin);
}

inline bool validateTriangle3D_Opt(size_t t, MeshHelper::TriMesh &delaunay, Matrix<double> &points, Config &config)
{

    auto &normals = delaunay.triangle_normals;
    auto normal = &normals[t * 3];

    auto prod = std::abs(dotProduct3(normal, config.desiredVector));
    if (prod < config.normThreshMin)
        return false;

    auto passZThresh = checkZThresh(t, delaunay, points, config.desiredVector, config.zThresh);
    return prod > config.normThresh || passZThresh;
}

inline bool validateTriangle4D(size_t t, MeshHelper::HalfEdgeTriangulation &delaunay, Matrix<double> &points, Config &config)
{
    // hmm simple for right now
    return checkPointClass(t, delaunay, points, config.allowedClass);
}

void createTriSet2(std::vector<uint8_t> &triSet, MeshHelper::HalfEdgeTriangulation &delaunay, Matrix<double> &points, Config &config)
{
    size_t numTriangles = delaunay.triangles.size() / 3;
// Ensure that each thread has at least PL_OMP_ELEM_PER_THREAD_TRISET
// Experimentation has found that too many threads will kill this loop if not enough work is presetn
#if defined(_OPENMP)
    int num_threads = std::min(omp_get_max_threads(), static_cast<int>(numTriangles / PL_OMP_ELEM_PER_THREAD_TRISET));
#pragma omp parallel for schedule(static, PL_OMP_CHUNK_SIZE_TRISET) num_threads(num_threads)
#endif
    for (size_t t = 0; t < numTriangles; t++)
    {
        triSet[t] = validateTriangle2D(t, delaunay, points, config) ? config.normalID : ZERO_UINT8;
    }
}

void createTriSet3(std::vector<uint8_t> &triSet, MeshHelper::HalfEdgeTriangulation &delaunay, Matrix<double> &points, Config &config)
{
    size_t numTriangles = delaunay.triangles.size() / 3;

// Ensure that each thread has at least PL_OMP_ELEM_PER_THREAD_TRISET
// Experimentation has found that too many threads will kill this loop if not enough work is presetn
#if defined(_OPENMP)
    int num_threads = std::min(omp_get_max_threads(), static_cast<int>(numTriangles / PL_OMP_ELEM_PER_THREAD_TRISET));
#pragma omp parallel for schedule(static, PL_OMP_CHUNK_SIZE_TRISET) num_threads(num_threads)
#endif
    for (size_t t = 0; t < numTriangles; t++)
    {
        if (triSet[t] != ZERO_UINT8)
            continue;
        uint8_t valid2D = validateTriangle2D(t, delaunay, points, config) ? ZERO_UINT8 : MAX_UINT8;
        uint8_t valid3D = validateTriangle3D(t, delaunay, points, config) ? config.normalID : ZERO_UINT8;
        triSet[t] = valid2D | valid3D;
    }
}

void createTriSet3_Opt(std::vector<uint8_t> &triSet, MeshHelper::TriMesh &delaunay, Matrix<double> &points, Config &config)
{
    size_t numTriangles = delaunay.triangles.size() / 3;

// Ensure that each thread has at least PL_OMP_ELEM_PER_THREAD_TRISET
// Experimentation has found that too many threads will kill this loop if not enough work is present
#if defined(_OPENMP)
    int num_threads = std::min(omp_get_max_threads(), static_cast<int>(numTriangles / PL_OMP_ELEM_PER_THREAD_TRISET));
#pragma omp parallel for schedule(static, PL_OMP_CHUNK_SIZE_TRISET) num_threads(num_threads)
#endif
    for (size_t t = 0; t < numTriangles; t++)
    {
        if (triSet[t] != ZERO_UINT8)
            continue;
        uint8_t valid2D = validateTriangle2D(t, delaunay, points, config) ? ZERO_UINT8 : MAX_UINT8;
        uint8_t valid3D = validateTriangle3D_Opt(t, delaunay, points, config) ? config.normalID : ZERO_UINT8;
        triSet[t] = valid2D | valid3D;
    }
}

void createTriSet3_Opt2(std::vector<uint8_t> &triSet, MeshHelper::TriMesh &delaunay, Matrix<double> &points, Config &config)
{
    size_t numTriangles = delaunay.triangles.size() / 3;

// Ensure that each thread has at least PL_OMP_ELEM_PER_THREAD_TRISET
// Experimentation has found that too many threads will kill this loop if not enough work is present
#if defined(_OPENMP)
    int num_threads = std::min(omp_get_max_threads(), static_cast<int>(numTriangles / PL_OMP_ELEM_PER_THREAD_TRISET));
#pragma omp parallel for schedule(static, PL_OMP_CHUNK_SIZE_TRISET) num_threads(num_threads)
#endif
    for (size_t t = 0; t < numTriangles; t++)
    {
        if (triSet[t] != ZERO_UINT8)
            continue;
        uint8_t valid2D = validateTriangle2D(t, delaunay, points, config) ? ZERO_UINT8 : MAX_UINT8;
        uint8_t valid3D = validateTriangle3D_Opt(t, delaunay, points, config) ? config.normalID : ZERO_UINT8;
        triSet[t] = valid2D | valid3D;
    }
}

void createTriSet4(std::vector<uint8_t> &triSet, MeshHelper::HalfEdgeTriangulation &delaunay, Matrix<double> &points, Config &config)
{
    size_t numTriangles = delaunay.triangles.size() / 3;
#if defined(_OPENMP)
    int num_threads = std::min(omp_get_max_threads(), static_cast<int>(numTriangles / PL_OMP_ELEM_PER_THREAD_TRISET));
#pragma omp parallel for schedule(static, PL_OMP_CHUNK_SIZE_TRISET) num_threads(num_threads)
#endif
    for (size_t t = 0; t < numTriangles; t++)
    {
        if (triSet[t] != ZERO_UINT8)
            continue;
        uint8_t valid2D = validateTriangle2D(t, delaunay, points, config) ? ZERO_UINT8 : MAX_UINT8;
        uint8_t valid4D = validateTriangle4D(t, delaunay, points, config) ? ZERO_UINT8 : MAX_UINT8;
        uint8_t valid3D = validateTriangle3D(t, delaunay, points, config) ? config.normalID : ZERO_UINT8;
        triSet[t] = valid2D | valid4D | valid3D;
    }
}

void constructPointHash(std::vector<size_t> &plane, MeshHelper::HalfEdgeTriangulation &delaunay, Matrix<double> &points,
                        polylidar::unordered_map<size_t, std::vector<size_t>> &pointHash, polylidar::unordered_map<size_t, size_t> &edgeHash,
                        ExtremePoint &xPoint, Config &config)
{
    auto &triangles = delaunay.triangles;
    auto &halfedges = delaunay.halfedges;

    size_t max_triangles_all = static_cast<size_t>(delaunay.triangles.size() / 3);
    std::vector<uint8_t> triSet(max_triangles_all, false);

    size_t max_triangles = static_cast<size_t>(plane.size());

    // This does not seem to make much of a difference
    // But it does not hurt it, Perimeter (boundary edges) grows as sqrt of area (triangles)
    size_t nominal_edges = static_cast<size_t>(std::sqrt(max_triangles) * 3);
    pointHash.reserve(nominal_edges);
    edgeHash.reserve(nominal_edges);

    // auto before = std::chrono::high_resolution_clock::now();
    // create a hash of all triangles in this plane set
    for (auto &&t : plane)
    {
        triSet[t] = true;
    }
    // auto after = std::chrono::high_resolution_clock::now();
    // auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(after - before);
    // std::cout << "ConstructPointHash - Time creating triangle Hash (ms): " << elapsed.count() << std::endl;

    // Loop through every triangle in the plane
    for (auto &&t : plane)
    {
        // Loop through every edge in the triangle
        for (int i = 0; i < 3; i++)
        {
            // get halfedge index
            auto heIndex = t * 3 + i;
            // get the adjacent edge of this triangle edge
            auto oppHe = halfedges[heIndex];
            // if (oppHe == INVALID_INDEX)
            //     continue;
            size_t oppT = static_cast<size_t>(oppHe / 3);
            // check if this triangle (oppT) is on the convex hull or removed
            if (oppHe == INVALID_INDEX || !triSet[oppT])
            {
                // Record this edge
                edgeHash[heIndex] = heIndex;
                // get point index of this half edge, this is an edge leaving from this pi
                auto pi = triangles[heIndex];
                trackExtremePoint(pi, points, xPoint, heIndex, config.rotationMatrix, config.needRotation);
                // Check if the point has already been indexed
                if (pointHash.find(pi) == pointHash.end())
                {
                    // construct a new vector holding this half edge index
                    // pointHash.insert(std::make_pair(pi, std::vector<size_t>(1, heIndex))); // No improvement
                    pointHash[pi] = std::vector<size_t>(1, heIndex);
                }
                else
                {
                    // point already exists, just append to it
                    pointHash[pi].push_back(heIndex);
                }
            }
        }
    }
}

std::vector<size_t> concaveSection(polylidar::unordered_map<size_t, std::vector<size_t>> &pointHash,
                                   polylidar::unordered_map<size_t, size_t> &edgeHash,
                                   MeshHelper::HalfEdgeTriangulation &delaunay,
                                   size_t startEdge, size_t stopPoint, Config &config,
                                   bool isHole)
{

    // std::cout << "Inside concave section" <<std::endl;
    std::vector<size_t> hullSection;

    auto &triangles = delaunay.triangles;
    // auto &coords = delaunay.coords;
    auto workingEdge = startEdge;
    // std::cout<< "Starting working edge: " << workingEdge << std::endl;
    while (true)
    {
        // std::this_thread::sleep_for(std::chrono::milliseconds(200));
        // std::cout << "Start while" << std::endl;
        edgeHash.erase(workingEdge);
        // Get the next EDGE of the SAME triangle
        auto nextHalfEdge = nextHalfedge(workingEdge);
        // std::cout << "nextHalf edge: " <<  nextHalfEdge <<std::endl;
        // Get the starting point of this edge
        auto nextPi = triangles[nextHalfEdge];
        // std::cout<< "nextPi: " << nextPi << std::endl;
        // std::cout<< "nextPi Coord: " << coords[2 * nextPi] << ", " << coords[2 * nextPi + 1] << std::endl;
        // Add point to the hull section
        hullSection.push_back(nextPi);
        // Check if this is the stop point
        if (nextPi == stopPoint)
        {
            return hullSection;
        }

        // Get outgoing edges for this point
        auto &nextEdges = pointHash[nextPi];

        if (nextEdges.size() == 0)
        {
            std::cerr << "ERROR! Found a broken edge when extracting a concave section (most likely during hole extraction). Possible that delaunator mislabeled an edge as part of the convex hull" << std::endl;
            // return empty hull
            return std::vector<size_t>();
        }

        // std::cout<< "nextEdges: " << nextEdges << std::endl;

        // filter edges that have already been seen!
        nextEdges.erase(std::remove_if(nextEdges.begin(), nextEdges.end(),
                                       [&edgeHash](size_t &e) { return edgeHash.count(e) == 0; }),
                        nextEdges.end());
        // std::cout<< "nextEdges after filter: " << nextEdges << std::endl;
        if (nextEdges.size() == 1)
        {
            workingEdge = nextEdges[0];
        }
        else
        {
            // We have a junction of outgoing edges at this point
            // std::cout << "Multiple Outgoing Edges. At Working Edge: " << workingEdge << "; At nextPi: " << nextPi << std::endl;
            auto workingEdgeVector = getVector(workingEdge, delaunay, config.rotationMatrix, config.needRotation, true);
            // std::cout << "Working Edge Vector " << PL_PRINT_ARRAY2(workingEdgeVector) << std::endl;
            workingEdge = getHullEdge(workingEdgeVector, nextEdges, delaunay, config.rotationMatrix, config.needRotation, isHole);
            // workingEdge = newEdge;
            // std::cout << "New Edge: " << newEdge << "; Next PI will be: " << triangles[newEdge] << std::endl;
        }
    }

    return hullSection;
}

std::vector<std::vector<size_t>> extractInteriorHoles(polylidar::unordered_map<size_t, std::vector<size_t>> pointHash,
                                                      polylidar::unordered_map<size_t, size_t> edgeHash,
                                                      MeshHelper::HalfEdgeTriangulation &delaunay, Config &config)
{
    std::vector<std::vector<size_t>> allHoles;
    auto &triangles = delaunay.triangles;
    // std::cout<< "Starting extracting Interior Holes" << std::endl;
    while (true)
    {
        if (edgeHash.empty())
        {
            break;
        }
        auto startEdge = std::begin(edgeHash)->first;
        // auto startingPointIndex = triangles[startEdge];
        auto stopPoint = triangles[startEdge];
        auto hole = concaveSection(pointHash, edgeHash, delaunay, startEdge, stopPoint, config, false);
        if (hole.size() > 0)
        {
            allHoles.push_back(hole);
        }
    }

    return allHoles;
}

Polygon extractConcaveHull(std::vector<size_t> &plane, MeshHelper::HalfEdgeTriangulation &delaunay, Matrix<double> &points, Config &config)
{
    Polygon poly;
    // point hash map
    polylidar::unordered_map<size_t, std::vector<size_t>> pointHash;
    // hash of all empty border half edges
    polylidar::unordered_map<size_t, size_t> edgeHash;
    // the left and right most extreme points
    ExtremePoint xPoint;

    // 99.6% of the time inside extractConcaveHull is inside constructPointHash
    // auto before = std::chrono::high_resolution_clock::now();
    constructPointHash(plane, delaunay, points, pointHash, edgeHash, xPoint, config);
    // auto after = std::chrono::high_resolution_clock::now();
    // auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(after - before);
    // std::cout << "ConstructPointHash - Total (ms): " << elapsed.count() << std::endl;

    // std::vector<size_t> bucket_sizes;
    // for(auto i = 0; i < edgeHash.bucket_count(); ++i)
    // {
    //   auto bucket_size = edgeHash.bucket_size(i);
    //   bucket_sizes.push_back(bucket_size);
    // }
    // std::cout << bucket_sizes << std::endl;

    auto startingHalfEdge = xPoint.xr_he;
    // Error checking, just in case the extreme right point has a hole connected to it
    auto &nextEdges = pointHash[xPoint.xr_pi];
    if (nextEdges.size() > 1)
    {
        // std::cout << "Starting half edge has a hole on it " << std::endl;
        // std::cout << "Right extreme point is connected to a hole. Determining correct edge..." << std::endl;
        // std::cout << "xPoint: " << xPoint << std::endl;
        // std::cout << "Plane size: " << plane.size() << std::endl;
        // std::cout << "Point Hash size: " << pointHash.size() << std::endl;
        // std::cout << "Edge Hash size: " << edgeHash.size() << std::endl;
        startingHalfEdge = getHullEdge(polylidar::UP_VECTOR, nextEdges, delaunay, config.rotationMatrix, config.needRotation, false);
    }
    auto startingPointIndex = xPoint.xr_pi;
    // std::cout << "Staring point index " << startingPointIndex << std::endl;;
    auto stopPoint = startingPointIndex;
    auto shell = concaveSection(pointHash, edgeHash, delaunay, startingHalfEdge, stopPoint, config, false);
    auto holes = extractInteriorHoles(pointHash, edgeHash, delaunay, config);

    holes.erase(std::remove_if(holes.begin(), holes.end(),
                               [&config](std::vector<size_t> &hole) { return hole.size() < config.minHoleVertices; }),
                holes.end());

    // std::vector<std::vector<size_t>> holes;
    poly.shell = std::move(shell);
    poly.holes = std::move(holes);
    return poly;
}

std::vector<Polygon> extractConcaveHulls(std::vector<std::vector<size_t>> planes, MeshHelper::HalfEdgeTriangulation &delaunay, Matrix<double> &points, Config &config)
{

    std::vector<Polygon> polygons;
    for (auto &&plane : planes)
    {
        Polygon poly = extractConcaveHull(plane, delaunay, points, config);
        polygons.push_back(std::move(poly));
    }
    return polygons;
}

void extractMeshSet(MeshHelper::HalfEdgeTriangulation &delaunay, std::vector<uint8_t> &triSet, size_t seedIdx, std::vector<size_t> &candidates, uint8_t &normalID)
{
    // Construct queue for triangle neighbor expansion
    std::queue<size_t> queue;
    // Add seed index to queue and erase from hash map
    queue.push(seedIdx);
    triSet[seedIdx] = MAX_UINT8;

    // std::vector<size_t> candidates;
    while (!queue.empty())
    {
        auto tri = queue.front();
        queue.pop();
        candidates.push_back(tri);
        // Get all neighbors that are inside our triangle hash map
        // Loop through every edge of this triangle and get adjacent triangle of this edge
        for (size_t i = 0; i < 3; ++i)
        {
            auto e = tri * 3 + i;
            auto opposite = delaunay.halfedges[e];
            if (opposite != INVALID_INDEX)
            {
                // convert opposite edge to a triangle
                size_t tn = opposite / 3;
                if (triSet[tn] == normalID)
                {
                    queue.push(tn);
                    triSet[tn] = MAX_UINT8;
                }
            }
        }
    }
}

bool passPlaneConstraints(std::vector<size_t> planeMesh, Config &config)
{
    return planeMesh.size() >= config.minTriangles;
}

std::vector<std::vector<size_t>> extractPlanesSet(MeshHelper::HalfEdgeTriangulation &delaunay, Matrix<double> &points, Config &config)
{
    std::vector<std::vector<size_t>> planes;
    size_t max_triangles = static_cast<size_t>(delaunay.triangles.size() / 3);
    std::vector<uint8_t> triSet(max_triangles, false);
    // auto before = std::chrono::high_resolution_clock::now();
    if (config.dim == 2)
    {
        createTriSet2(triSet, delaunay, points, config);
    }
    else if (config.dim == 3)
    {
        createTriSet3(triSet, delaunay, points, config);
    }
    else if (config.dim == 4)
    {
        createTriSet4(triSet, delaunay, points, config);
    }
    // auto after = std::chrono::high_resolution_clock::now();
    // float elapsed_d = static_cast<std::chrono::duration<float, std::milli>>(after - before).count();
    // std::cout << "CreateTriSet took " << elapsed_d << " milliseconds" << std::endl;

    for (size_t t = 0; t < max_triangles; t++)
    {
        if (triSet[t] == config.normalID)
        {
            planes.emplace_back();                       // construct empty vector inside planes
            auto &planeMesh = planes[planes.size() - 1]; // retrieve this newly created vector
            extractMeshSet(delaunay, triSet, t, planeMesh, config.normalID);
            if (!passPlaneConstraints(planeMesh, config))
            {
                planes.pop_back();
            }
        }
    }

    return planes;
}

// Slightly more optimized Plane Extraction for 3D triangular meshes
// Triangle normals are already computed, or computed in bulk before creating the TriSet - triangles passing set consraints
std::vector<std::vector<size_t>> extractPlanesSet(MeshHelper::TriMesh &delaunay, Matrix<double> &points, Config &config)
{
    std::vector<std::vector<size_t>> planes;
    size_t max_triangles = static_cast<size_t>(delaunay.triangles.size() / 3);
    std::vector<uint8_t> triSet(max_triangles, 0);
    if (delaunay.triangle_normals.size() <= 0)
    {
        MeshHelper::ComputeTriangleNormals(delaunay.coords, delaunay.triangles, delaunay.triangle_normals);
    }
    createTriSet3_Opt2(triSet, delaunay, points, config);

    for (size_t t = 0; t < max_triangles; t++)
    {
        if (triSet[t] == config.normalID)
        {

            planes.emplace_back();                       // construct empty vector inside planes
            auto &planeMesh = planes[planes.size() - 1]; // retrieve this newly created vector
            extractMeshSet(delaunay, triSet, t, planeMesh, config.normalID);
            if (!passPlaneConstraints(planeMesh, config))
            {
                planes.pop_back();
            }
        }
    }

    return planes;
}

// Slightly more optimized Plane Extraction for 3D triangular meshes
// Triangle normals are already computed, or computed in bulk before creating the TriSet - triangles passing set consraints
std::vector<std::vector<size_t>> extractPlanesSetMultipleNormals(MeshHelper::TriMesh &delaunay, Matrix<double> &points, std::vector<uint8_t> &triSet, Config &config)
{
    std::vector<std::vector<size_t>> planes;
    size_t max_triangles = static_cast<size_t>(delaunay.triangles.size() / 3);
    if (delaunay.triangle_normals.size() <= 0)
    {
        MeshHelper::ComputeTriangleNormals(delaunay.coords, delaunay.triangles, delaunay.triangle_normals);
    }
    createTriSet3_Opt2(triSet, delaunay, points, config);

    for (size_t t = 0; t < max_triangles; t++)
    {
        if (triSet[t] == config.normalID)
        {

            planes.emplace_back();                       // construct empty vector inside planes
            auto &planeMesh = planes[planes.size() - 1]; // retrieve this newly created vector
            extractMeshSet(delaunay, triSet, t, planeMesh, config.normalID);
            if (!passPlaneConstraints(planeMesh, config))
            {
                planes.pop_back();
            }
        }
    }

    return planes;
}

std::tuple<delaunator::Delaunator, std::vector<std::vector<size_t>>, std::vector<Polygon>> ExtractPlanesAndPolygons(Matrix<double> &nparray, Config config)
{
    config.dim = nparray.cols;
    Timer timer(true);
    // auto before = std::chrono::high_resolution_clock::now();
    delaunator::Delaunator delaunay(nparray);
    // std::cout << "Delaunay init took: " << timer  << std::endl;
    delaunay.triangulate();
    // std::cout << "Delaunay took: " << timer  << std::endl;
    // auto after = std::chrono::high_resolution_clock::now();
    // auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(after - before);
    // std::cout << "Delaunay took " << elapsed.count() << " milliseconds" << std::endl;

    // before = std::chrono::high_resolution_clock::now();
    timer.Reset();
    std::vector<std::vector<size_t>> planes = extractPlanesSet(delaunay, nparray, config);
    // std::cout << "Plane Extraction took: " << timer  << std::endl;
    // after = std::chrono::high_resolution_clock::now();
    // elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(after - before);
    // std::cout << "Plane Extraction took " << elapsed.count() << " milliseconds" << std::endl;

    // before = std::chrono::high_resolution_clock::now();
    std::vector<Polygon> polygons = extractConcaveHulls(planes, delaunay, nparray, config);
    // after = std::chrono::high_resolution_clock::now();
    // elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(after - before);
    // std::cout << "Polygon Hull Extraction took " << elapsed.count() << " milliseconds" << std::endl;
    return std::make_tuple(delaunay, planes, polygons);

    // nparray2D is a contigious buffer of (ROWS,2)
}

std::vector<double> ExtractNormalsFromMesh(MeshHelper::TriMesh &triangulation, Config &config)
{
    auto &vertices = triangulation.coords;
    std::vector<double> triangle_normals;
    config.dim = vertices.cols;
    MeshHelper::ComputeTriangleNormals(vertices, triangulation.triangles, triangle_normals);
    triangulation.triangle_normals.swap(triangle_normals);
    return triangle_normals;
}

std::tuple<std::vector<std::vector<size_t>>, std::vector<Polygon>> ExtractPlanesAndPolygonsFromMesh(MeshHelper::TriMesh &triangulation, Config config)
{
    auto &vertices = triangulation.coords;
    config.dim = vertices.cols;
    // Create rotation matrix
    UpdateConfigWithRotationInformation(config);
    // auto t0 = std::chrono::high_resolution_clock::now();
    std::vector<std::vector<size_t>> planes = extractPlanesSet(triangulation, vertices, config);
    // auto t1 = std::chrono::high_resolution_clock::now();
    // float elapsed_d = static_cast<std::chrono::duration<float, std::milli>>(t1 - t0).count();
    // std::cout << "Plane Extraction took " << elapsed_d << " milliseconds" << std::endl;
    std::vector<Polygon> polygons = extractConcaveHulls(planes, triangulation, vertices, config);
    // auto t2 = std::chrono::high_resolution_clock::now();
    // elapsed_d = static_cast<std::chrono::duration<float, std::milli>>(t2 - t1).count();
    // std::cout << "Polygon Hull Extraction took " << elapsed_d << " milliseconds" << std::endl;
    // I think the std move is what I'm looking for??
    return std::make_tuple(std::move(planes), std::move(polygons));
}



std::vector<Polygon> ExtractPolygonsFromMesh(MeshHelper::TriMesh &triangulation, Config config)
{
    auto &vertices = triangulation.coords;
    config.dim = vertices.cols;
    UpdateConfigWithRotationInformation(config);
    std::vector<std::vector<size_t>> planes = extractPlanesSet(triangulation, vertices, config);
    std::vector<Polygon> polygons = extractConcaveHulls(planes, triangulation, vertices, config);
    return polygons;
}

std::vector<std::vector<Polygon>> ExtractPolygonsFromMesh(MeshHelper::TriMesh &triangulation, const Matrix<double> &normals, const Config &config)
{
    auto &vertices = triangulation.coords;
    std::vector<Config> configs = CreateMultipleConfigsFromNormals(config, normals);
    size_t max_triangles = static_cast<size_t>(triangulation.triangles.size() / 3);
    std::vector<uint8_t> triSet(max_triangles, 0);
    std::vector<std::vector<Polygon>> all_polygons;
    for (auto && config_: configs)
    {
        std::vector<std::vector<size_t>> planes = extractPlanesSetMultipleNormals(triangulation, vertices, triSet, config_);
        std::vector<Polygon> polygons = extractConcaveHulls(planes, triangulation, vertices, config_);
        all_polygons.emplace_back(std::move(polygons));
    }

    return all_polygons;
}
// std::vector<Polygon> ExtractPolygonsFromMesh(MeshHelper::TriMesh &triangulation, Config config, const Matrix<double> &normals)
// {

// }

std::vector<Polygon> ExtractPolygons(Matrix<double> &nparray, Config config)
{
    config.dim = nparray.cols;

    // auto before = std::chrono::high_resolution_clock::now();
    delaunator::Delaunator delaunay(nparray);
    delaunay.triangulate();
    // auto after = std::chrono::high_resolution_clock::now();
    // float elapsed_d = static_cast<std::chrono::duration<float, std::milli>>(after - before).count();
    // std::cout << "Delaunay took " << elapsed_d << " milliseconds" << std::endl;

    // before = std::chrono::high_resolution_clock::now();
    std::vector<std::vector<size_t>> planes = extractPlanesSet(delaunay, nparray, config);
    // after = std::chrono::high_resolution_clock::now();
    // float elapsed_ep = static_cast<std::chrono::duration<float, std::milli>>(after - before).count();
    // std::cout << "Plane Extraction took " << elapsed_ep << " milliseconds" << std::endl;

    // before = std::chrono::high_resolution_clock::now();
    std::vector<Polygon> polygons = extractConcaveHulls(planes, delaunay, nparray, config);
    // after = std::chrono::high_resolution_clock::now();
    // float elapsed_ch = static_cast<std::chrono::duration<float, std::milli>>(after - before).count();
    // std::cout << "Polygon Hull Extraction took " << elapsed_ch << " milliseconds" << std::endl;
    return polygons;
}

std::vector<Polygon> ExtractPolygonsAndTimings(Matrix<double> &nparray, Config config, std::vector<float> &timings)
{
    config.dim = nparray.cols;

    auto before = std::chrono::high_resolution_clock::now();
    delaunator::Delaunator delaunay(nparray);
    delaunay.triangulate();
    auto after = std::chrono::high_resolution_clock::now();
    float elapsed_d = static_cast<std::chrono::duration<float, std::milli>>(after - before).count();
    // std::cout << "Delaunay took " << elapsed_d << " milliseconds" << std::endl;

    before = std::chrono::high_resolution_clock::now();
    std::vector<std::vector<size_t>> planes = extractPlanesSet(delaunay, nparray, config);
    after = std::chrono::high_resolution_clock::now();
    float elapsed_ep = static_cast<std::chrono::duration<float, std::milli>>(after - before).count();
    // std::cout << "Plane Extraction took " << elapsed_ep << " milliseconds" << std::endl;

    // std::vector<Polygon> polygons;
    // float elapsed_ch = 0.0;

    before = std::chrono::high_resolution_clock::now();
    std::vector<Polygon> polygons = extractConcaveHulls(planes, delaunay, nparray, config);
    after = std::chrono::high_resolution_clock::now();
    float elapsed_ch = static_cast<std::chrono::duration<float, std::milli>>(after - before).count();
    // std::cout << "Polygon Hull Extraction took " << elapsed_ch << " milliseconds" << std::endl;
    timings.push_back(elapsed_d);
    timings.push_back(elapsed_ep);
    timings.push_back(elapsed_ch);

    return polygons;
}

} // namespace polylidar