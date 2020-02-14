#include "polylidar/polylidar.hpp"


namespace polylidar
{

inline bool validateTriangle2D(size_t t, delaunator::HalfEdgeTriangulation &delaunay, Matrix<double> &points, Config &config)
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

inline bool validateTriangle3D(size_t t, delaunator::HalfEdgeTriangulation &delaunay, Matrix<double> &points, Config &config)
{
    double zDiff = 0.0;
    std::array<double, 3> normal;
    // get zDiff and normal of triangle
    maxZChangeAndNormal(t, delaunay, points, zDiff, normal, config.desiredVector);
    // get dot product of triangle
    auto prod = std::abs(dotProduct3(normal, config.desiredVector));

    bool passZThresh = config.zThresh > 0 && zDiff < config.zThresh;

    return prod > config.normThresh || (passZThresh && prod > config.normThreshMin);
}

inline bool validateTriangle4D(size_t t, delaunator::HalfEdgeTriangulation &delaunay, Matrix<double> &points, Config &config)
{
    // hmm simple for right now
    return checkPointClass(t, delaunay, points, config.allowedClass);
}

void createTriSet2(std::vector<bool> &triSet, delaunator::HalfEdgeTriangulation &delaunay, Matrix<double> &points, Config &config)
{
    size_t numTriangles = std::floor(delaunay.triangles.size() / 3);
    // Ensure that each thread has at least PL_OMP_ELEM_PER_THREAD_TRISET
    // Experimentation has found that too many threads will kill this loop if not enough work is presetn
    #if defined(_OPENMP)
    int num_threads = std::min(omp_get_max_threads(), static_cast<int>(numTriangles / PL_OMP_ELEM_PER_THREAD_TRISET));
    #pragma omp parallel for schedule(static, PL_OMP_CHUNK_SIZE_TRISET) num_threads(num_threads)
    #endif
    for (size_t t = 0; t < numTriangles; t++)
    {
        if (validateTriangle2D(t, delaunay, points, config))
        {
            triSet[t] = true;
        }
    }
}

void createTriSet3(std::vector<bool> &triSet, delaunator::HalfEdgeTriangulation &delaunay, Matrix<double> &points, Config &config)
{
    size_t numTriangles = std::floor(delaunay.triangles.size() / 3);

    // Ensure that each thread has at least PL_OMP_ELEM_PER_THREAD_TRISET
    // Experimentation has found that too many threads will kill this loop if not enough work is presetn
    #if defined(_OPENMP)
    int num_threads = std::min(omp_get_max_threads(), static_cast<int>(numTriangles / PL_OMP_ELEM_PER_THREAD_TRISET));
    #pragma omp parallel for schedule(static, PL_OMP_CHUNK_SIZE_TRISET) num_threads(num_threads)
    #endif
    for (size_t t = 0; t < numTriangles; t++)
    {
        bool valid2D = validateTriangle2D(t, delaunay, points, config);
        bool valid3D = validateTriangle3D(t, delaunay, points, config);
        triSet[t] = valid2D && valid3D;
        // triSet[t] = points(delaunay.triangles[t *3], 2) < 100.0;
    }
}

void createTriSet4(std::vector<bool> &triSet, delaunator::HalfEdgeTriangulation &delaunay, Matrix<double> &points, Config &config)
{
    size_t numTriangles = std::floor(delaunay.triangles.size() / 3);
    #if defined(_OPENMP)
    int num_threads = std::min(omp_get_max_threads(), static_cast<int>(numTriangles / PL_OMP_ELEM_PER_THREAD_TRISET));
    #pragma omp parallel for schedule(static, PL_OMP_CHUNK_SIZE_TRISET) num_threads(num_threads)
    #endif
    for (size_t t = 0; t < numTriangles; t++)
    {
        bool valid2D = validateTriangle2D(t, delaunay, points, config);
        bool valid3D = validateTriangle3D(t, delaunay, points, config);
        bool valid4D = validateTriangle4D(t, delaunay, points, config);
        if (valid2D && valid3D && valid4D)
        {
            triSet[t] = true;
        }
    }
}

void constructPointHash(std::vector<size_t> &plane, delaunator::HalfEdgeTriangulation &delaunay, Matrix<double> &points,
                        polylidar::unordered_map<size_t, std::vector<size_t>> &pointHash, polylidar::unordered_map<size_t, size_t> &edgeHash,
                        ExtremePoint &xPoint, Config &config)
{
    auto &triangles = delaunay.triangles;
    auto &halfedges = delaunay.halfedges;

    size_t max_triangles_all = static_cast<size_t>(delaunay.triangles.size() / 3);
    std::vector<bool> triSet(max_triangles_all, false);

    // THIS REDUCED THIS FUNCTIONS RUNTIME BY 1/2
    // LESS ALLOCATIONS AND HASH COLLISIONS
    size_t max_triangles = static_cast<size_t>(plane.size());
    // triHash.reserve(max_triangles);

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
                                   delaunator::HalfEdgeTriangulation &delaunay,
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
                                                      delaunator::HalfEdgeTriangulation &delaunay, Config &config)
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

Polygon extractConcaveHull(std::vector<size_t> &plane, delaunator::HalfEdgeTriangulation &delaunay, Matrix<double> &points, Config &config)
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

std::vector<Polygon> extractConcaveHulls(std::vector<std::vector<size_t>> planes, delaunator::HalfEdgeTriangulation &delaunay, Matrix<double> &points, Config &config)
{

    std::vector<Polygon> polygons;
    for (auto &&plane : planes)
    {
        Polygon poly = extractConcaveHull(plane, delaunay, points, config);
        polygons.push_back(std::move(poly));
    }
    return polygons;
}

void extractMeshSet(delaunator::HalfEdgeTriangulation &delaunay, std::vector<bool> &triSet, size_t seedIdx, std::vector<size_t> &candidates)
{
    // Construct queue for triangle neighbor expansion
    std::queue<size_t> queue;
    // Add seed index to queue and erase from hash map
    queue.push(seedIdx);
    triSet[seedIdx] = false;

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
                size_t tn = std::floor(opposite / 3);
                if (triSet[tn])
                {
                    queue.push(tn);
                    triSet[tn] = false;
                }
            }
        }
    }
}

// TODO
bool passPlaneConstraints(std::vector<size_t> planeMesh, delaunator::HalfEdgeTriangulation &delaunay, Config &config)
{
    if (planeMesh.size() < config.minTriangles)
    {
        return false;
    }
    return true;
}

std::vector<std::vector<size_t>> extractPlanesSet(delaunator::HalfEdgeTriangulation &delaunay, Matrix<double> &points, Config &config)
{
    std::vector<std::vector<size_t>> planes;
    size_t max_triangles = static_cast<size_t>(delaunay.triangles.size() / 3);
    std::vector<bool> triSet(max_triangles, false);
    auto before = std::chrono::high_resolution_clock::now();
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
    auto after = std::chrono::high_resolution_clock::now();
    float elapsed_d = std::chrono::duration_cast<std::chrono::microseconds>(after - before).count() * 1e-3;
    // std::cout << "CreateTriSet took " << elapsed_d << " milliseconds" << std::endl;

    for (size_t t = 0; t < max_triangles; t++)
    {
        if (triSet[t])
        {
            planes.emplace_back();                       // construct empty vector inside planes
            auto &planeMesh = planes[planes.size() - 1]; // retrieve this newly created vector
            extractMeshSet(delaunay, triSet, t, planeMesh);
            // Remove plane if it does not pass constraints
            if (!passPlaneConstraints(planeMesh, delaunay, config))
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

    // auto before = std::chrono::high_resolution_clock::now();
    delaunator::Delaunator delaunay(nparray);
    delaunay.triangulate();
    // auto after = std::chrono::high_resolution_clock::now();
    // auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(after - before);
    // std::cout << "Delaunay took " << elapsed.count() << " milliseconds" << std::endl;

    // before = std::chrono::high_resolution_clock::now();
    std::vector<std::vector<size_t>> planes = extractPlanesSet(delaunay, nparray, config);
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


std::tuple<std::vector<std::vector<size_t>>, std::vector<Polygon>> ExtractPlanesAndPolygonsFromMesh(delaunator::HalfEdgeTriangulation &triangulation, Config config)
{
    auto &vertices = triangulation.coords;
    config.dim = vertices.cols;
    // Create rotation matrix
    std::array<double,3> axis;
    double angle;
    // std::cout << "Normal to Extract: " << PL_PRINT_ARRAY(config.desiredVector) << "; Z Axis: " << PL_PRINT_ARRAY(DEFAULT_DESIRED_VECTOR) << std::endl;
    std::tie(axis, angle) = axisAngleFromVectors(config.desiredVector, DEFAULT_DESIRED_VECTOR);
    if (std::abs(angle) > EPS_RADIAN)
    {
        config.needRotation = true;
        config.rotationMatrix = axisAngleToRotationMatrix(axis, angle);
    }
    // std::cout << "Axis: " << PL_PRINT_ARRAY(axis) <<  "; angle: " << angle << std::endl;
    // print_matrix(config.rotationMatrix);


    auto t0 = std::chrono::high_resolution_clock::now();
    std::vector<std::vector<size_t>> planes = extractPlanesSet(triangulation, vertices, config);
    auto t1 = std::chrono::high_resolution_clock::now();
    float elapsed_d = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() * 1e-3;
    std::cout << "Plane Extraction took " << elapsed_d << " milliseconds" << std::endl;
    std::vector<Polygon> polygons = extractConcaveHulls(planes, triangulation, vertices, config);
    auto t2 = std::chrono::high_resolution_clock::now();
    elapsed_d = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() * 1e-3;
    std::cout << "Polygon Hull Extraction took " << elapsed_d << " milliseconds" << std::endl;
    // I think the std move is what I'm looking for??
    return std::make_tuple(std::move(planes), std::move(polygons));
}

std::vector<Polygon> ExtractPolygonsFromMesh(delaunator::HalfEdgeTriangulation &triangulation, Config config)
{
    auto vertices = triangulation.coords;
    config.dim = vertices.cols;

    std::vector<std::vector<size_t>> planes = extractPlanesSet(triangulation, vertices, config);
    std::vector<Polygon> polygons = extractConcaveHulls(planes, triangulation, vertices, config);
    return polygons;
}

std::vector<Polygon> ExtractPolygons(Matrix<double> &nparray, Config config)
{
    config.dim = nparray.cols;

    // auto before = std::chrono::high_resolution_clock::now();
    delaunator::Delaunator delaunay(nparray);
    delaunay.triangulate();
    // auto after = std::chrono::high_resolution_clock::now();
    // float elapsed_d = std::chrono::duration_cast<std::chrono::microseconds>(after - before).count() * 1e-3;
    // std::cout << "Delaunay took " << elapsed_d << " milliseconds" << std::endl;

    // before = std::chrono::high_resolution_clock::now();
    std::vector<std::vector<size_t>> planes = extractPlanesSet(delaunay, nparray, config);
    // after = std::chrono::high_resolution_clock::now();
    // float elapsed_ep = std::chrono::duration_cast<std::chrono::microseconds>(after - before).count() * 1e-3;
    // std::cout << "Plane Extraction took " << elapsed_ep << " milliseconds" << std::endl;

    // before = std::chrono::high_resolution_clock::now();
    std::vector<Polygon> polygons = extractConcaveHulls(planes, delaunay, nparray, config);
    // after = std::chrono::high_resolution_clock::now();
    // float elapsed_ch = std::chrono::duration_cast<std::chrono::microseconds>(after - before).count() * 1e-3;
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
    float elapsed_d = std::chrono::duration_cast<std::chrono::microseconds>(after - before).count() * 1e-3;
    // std::cout << "Delaunay took " << elapsed_d << " milliseconds" << std::endl;

    before = std::chrono::high_resolution_clock::now();
    std::vector<std::vector<size_t>> planes = extractPlanesSet(delaunay, nparray, config);
    after = std::chrono::high_resolution_clock::now();
    float elapsed_ep = std::chrono::duration_cast<std::chrono::microseconds>(after - before).count() * 1e-3;
    // std::cout << "Plane Extraction took " << elapsed_ep << " milliseconds" << std::endl;

    // std::vector<Polygon> polygons;
    // float elapsed_ch = 0.0;

    before = std::chrono::high_resolution_clock::now();
    std::vector<Polygon> polygons = extractConcaveHulls(planes, delaunay, nparray, config);
    after = std::chrono::high_resolution_clock::now();
    float elapsed_ch = std::chrono::duration_cast<std::chrono::microseconds>(after - before).count() * 1e-3;
    // std::cout << "Polygon Hull Extraction took " << elapsed_ch << " milliseconds" << std::endl;
    timings.push_back(elapsed_d);
    timings.push_back(elapsed_ep);
    timings.push_back(elapsed_ch);

    return polygons;
}

void deproject_points(const size_t i, const size_t j, float depth, const Matrix<double> &intrinsics, double &x, double &y, double &z)
{
    z = static_cast<double>(depth);
    x = (static_cast<double>(j) - intrinsics(0, 2)) * z / intrinsics(0, 0);
    y = (static_cast<double>(i) - intrinsics(1, 2)) * z / intrinsics(1, 1);
}

std::vector<double> ExtractPointCloudFromFloatDepth(const Matrix<float> &im, const Matrix<double> &intrinsics, const size_t stride)
{
    std::vector<double> points;
    auto rows = im.rows;
    auto cols = im.cols;
    size_t cols_stride = ceil(cols / float(stride));
    size_t rows_stride = ceil(rows / float(stride));
    points.resize(cols_stride * rows_stride * 3);
    // size_t pnt_cnt = 0;
    // #pragma omp parallel for schedule(static) // no speedup??
    for (size_t i = 0; i < rows; i += stride)
    {
        for (size_t j = 0; j < cols; j += stride)
        {
            size_t p_idx = static_cast<size_t>((cols_stride * i/stride + j/stride) * 3);
            deproject_points(i, j, im(i, j), intrinsics, points[p_idx], points[p_idx + 1], points[p_idx + 2]);
            // pnt_cnt++;
        }
    }
    // std::cout << "Point Count: " << pnt_cnt << "; Expected: "<< cols_stride * rows_stride <<std::endl;
    // std::cout << "extractPointCloudFromFloatDepth C++ : " << points[0] << " Address:" <<  &points[0] << std::endl;
    return points;
}

std::vector<size_t> ExtractHalfEdgesFromUniformMesh(size_t rows, size_t cols, std::vector<size_t> &triangles,
                                                    std::vector<size_t> &valid_tri, size_t stride)
{
    constexpr std::size_t INVALID_INDEX = std::numeric_limits<std::size_t>::max();
    std::vector<size_t> halfedges(triangles.size(), INVALID_INDEX);
    // This represents the number of rows and columns of the downsampled POINT CLOUD
    size_t cols_stride = ceil(cols / float(stride));
    size_t rows_stride = ceil(rows / float(stride));
    // This represent the number of rows and columns of the UNIFORM TRIANGULAR MESH
    size_t cols_tris = cols_stride - 1;
    size_t rows_tris = rows_stride - 1;
    // #pragma omp parallel for schedule(static) // No speedup, tried many settings
    for (size_t i = 0; i < rows_tris; i++)
    {
        // int tid = omp_get_thread_num();
        // std::cout << "Hello from " << tid << std::endl;
        for (size_t j = 0; j < cols_tris; j++)
        {
            // These are the triangle indexes in the global full mesh
            size_t t_global_idx_first = (cols_tris * i + j) * 2;
            size_t t_global_idx_second = (cols_tris * i + j) * 2 + 1;
            // We convert these global meshes to our valid mesh indices
            auto t_valid_idx_first = valid_tri[t_global_idx_first];
            auto t_valid_idx_second = valid_tri[t_global_idx_second];
            // Valid triangles indices bordering the first triangle
            size_t t_valid_idx_top = 0;
            size_t t_valid_idx_right = 0;
            // valid triangle indices bordering the second triangle
            size_t t_valid_idx_bottom = 0;
            size_t t_valid_idx_left = 0;
            // Check if first triangle is valid, if invalid its not in our mesh
            if (t_valid_idx_first != INVALID_INDEX)
            {
                // Check if we are on the top of the depth image
                if (i == 0)
                {
                    t_valid_idx_top = INVALID_INDEX; // indicates this edge has no border
                }
                else
                {
                    // gets the triangle one row up from this one, math is from implicit structure
                    size_t t_global_idx_top = t_global_idx_first - 2 * cols_tris + 1;
                    // convert to valid mesh index
                    t_valid_idx_top = valid_tri[t_global_idx_top];
                }
                // check if we are on the right side of the depth Image
                if (j >= cols_tris - 1)
                {
                    t_valid_idx_right = INVALID_INDEX; // indicates this edge has no border
                }
                else
                {
                    // gets the triangle one cell to the right, math is from implicit structure
                    size_t t_global_idx_right = t_global_idx_first + 3;
                    t_valid_idx_right = valid_tri[t_global_idx_right];
                }
                // Set the edges if they are valid
                if (t_valid_idx_top != INVALID_INDEX)
                    halfedges[size_t(t_valid_idx_first * 3)] = t_valid_idx_top * 3;
                if (t_valid_idx_right != INVALID_INDEX)
                    halfedges[size_t(t_valid_idx_first * 3 + 1)] = t_valid_idx_right * 3 + 1;
                if (t_valid_idx_second != INVALID_INDEX)
                    halfedges[size_t(t_valid_idx_first * 3 + 2)] = t_valid_idx_second * 3 + 2;
            }
            // Check if second triangle is valid, if invalid its not in our mesh
            if (t_valid_idx_second != INVALID_INDEX)
            {
                // We have a valid second triangle
                // Check if we are on the bottom of the depth image
                if (i == rows_tris - 1)
                {
                    t_valid_idx_bottom = INVALID_INDEX;
                }
                else
                {
                    size_t t_global_idx_bottom = t_global_idx_second + 2 * cols_tris - 1;
                    t_valid_idx_bottom = valid_tri[t_global_idx_bottom];
                }
                // Check if we are on the left side of the RGBD Image, if so than we have a border on the left
                if (j == 0)
                {
                    t_valid_idx_left = INVALID_INDEX;
                }
                else
                {
                    size_t t_global_idx_left = t_global_idx_second - 3;
                    t_valid_idx_left = valid_tri[t_global_idx_left];
                }
                // Set Edges
                if (t_valid_idx_bottom != INVALID_INDEX)
                    halfedges[size_t(t_valid_idx_second * 3)] = t_valid_idx_bottom * 3;
                if (t_valid_idx_left != INVALID_INDEX)
                    halfedges[size_t(t_valid_idx_second * 3 + 1)] = t_valid_idx_left * 3 + 1;
                if (t_valid_idx_first != INVALID_INDEX)
                    halfedges[size_t(t_valid_idx_second * 3 + 2)] = t_valid_idx_first * 3 + 2;
            }
        }
    }
    return halfedges;
}

std::tuple<std::vector<size_t>, std::vector<size_t>> CreateUniformMesh(size_t rows, size_t cols, std::vector<double> &points, size_t stride)
{
    Matrix<double> points_2D(points.data(), points.size() / 3, 3);
    std::vector<size_t> triangles;
    // This represents the number of rows and columns of the downsampled POINT CLOUD
    size_t cols_stride = ceil(cols / float(stride));
    size_t rows_stride = ceil(rows / float(stride));
    // This represent the number of rows and columns of the UNIFORM TRIANGULAR MESH
    size_t cols_tris = cols_stride - 1;
    size_t rows_tris = rows_stride - 1;
    // These are the maximum number of triangles that can ever be in the mesh
    size_t max_triangles = cols_tris * rows_tris * 2;
    // This will count valid points and triangles
    size_t tri_cnt = 0;
    size_t pix_cnt = 0;
    // Invalid Triangle Marker
    constexpr std::size_t INVALID_INDEX = std::numeric_limits<std::size_t>::max();
    std::vector<size_t> valid_tri(max_triangles, INVALID_INDEX);
    // Reserve memory for triangles
    triangles.reserve(max_triangles);
    for (size_t i = 0; i < rows_tris; i++)
    {
        for (size_t j = 0; j < cols_tris; j++)
        {
            size_t p1_idx = i * cols_stride + j;
            size_t p2_idx = i * cols_stride + j + 1;
            size_t p3_idx = (i + 1) * cols_stride + j + 1;
            size_t p4_idx = (i + 1) * cols_stride + j;

            auto &p1 = points_2D(p1_idx, 2);
            auto &p2 = points_2D(p2_idx, 2);
            auto &p3 = points_2D(p3_idx, 2);
            auto &p4 = points_2D(p4_idx, 2);

            if (p1 > 0 && p2 > 0 && p3 > 0)
            {
                triangles.push_back(p1_idx);
                triangles.push_back(p2_idx);
                triangles.push_back(p3_idx);
                valid_tri[pix_cnt * 2] = tri_cnt;
                tri_cnt++;
            }
            if (p3 > 0 && p4 > 0 && p1 > 0)
            {
                triangles.push_back(p3_idx);
                triangles.push_back(p4_idx);
                triangles.push_back(p1_idx);
                valid_tri[pix_cnt * 2 + 1] = tri_cnt;
                tri_cnt++;
            }
            pix_cnt++;
        }
    }
    return std::make_tuple(std::move(triangles), std::move(valid_tri));
}

std::tuple<std::vector<double>, std::vector<size_t>, std::vector<size_t>> ExtractUniformMeshFromFloatDepth(const Matrix<float> &im, const Matrix<double> &intrinsics, const size_t stride)
{
    std::vector<size_t> triangles;
    std::vector<size_t> valid_tri;
    auto t0 = std::chrono::high_resolution_clock::now();
    std::vector<double> points = ExtractPointCloudFromFloatDepth(im, intrinsics, stride);
    auto t1 = std::chrono::high_resolution_clock::now();
    float elapsed_d = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() * 1e-3;
    std::cout << "Point Cloud Extraction took " << elapsed_d << " milliseconds" << std::endl;
    std::tie(triangles, valid_tri) = CreateUniformMesh(im.rows, im.cols, points, stride);
    auto t2 = std::chrono::high_resolution_clock::now();
    elapsed_d = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() * 1e-3;
    std::cout << "Create Uniform Mesh took " << elapsed_d << " milliseconds" << std::endl;
    std::vector<size_t> halfedges = ExtractHalfEdgesFromUniformMesh(im.rows, im.cols, triangles, valid_tri, stride);
    auto t3 = std::chrono::high_resolution_clock::now();
    elapsed_d = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count() * 1e-3;
    std::cout << "Extract Half Edge took " << elapsed_d << " milliseconds" << std::endl;

    // std::cout << "extractUniformMeshFromFloatDepth C++ : " << points[0] << " Address:" <<  &points[0] << std::endl;
    return std::make_tuple(std::move(points), std::move(triangles), std::move(halfedges));
}



} // namespace polylidar