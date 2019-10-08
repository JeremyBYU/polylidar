#include "polylidar.hpp"

namespace polylidar
{

std::ostream &operator<<(std::ostream &os, const Config &config)
{
    os << "Dim=" << config.dim << " alpha=" << config.alpha << " xyThresh=" << config.xyThresh << " lmax=" << config.xyThresh << " minTriangles=" << config.minTriangles
       << " minBboxArea=" << config.minBboxArea << " zThresh=" << config.zThresh << " normThresh=" << config.normThresh
       << " allowedClass=" << config.allowedClass
       << " desiredVector= [" << (config.desiredVector)[0] << ", " << (config.desiredVector)[1] << ", " << (config.desiredVector)[2] << "]";

    return os;
}

std::ostream &operator<<(std::ostream &os, const std::vector<size_t> &values)
{
    os << "[";
    for (auto &&val : values)
    {
        os << val << ", ";
    }
    os << "]";

    return os;
}

std::ostream &operator<<(std::ostream &os, const ExtremePoint &values)
{
    os << "xr_he" << values.xr_he << " xr_pi" << values.xr_pi << " xr_val" << values.xr_val;

    return os;
}

void copy2Ddata(Matrix &src, std::vector<double> &dest)
{
    size_t rows = src.rows;

    for (size_t i = 0; i < rows; i++)
    {
        dest[2*i] = src(i,0);
        dest[2*i+1] = src(i,1);
    }
}

inline bool validateTriangle2D(size_t t, delaunator::Delaunator &delaunay, Matrix &points, Config &config)
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


inline bool validateTriangle3D(size_t t, delaunator::Delaunator &delaunay,  Matrix &points, Config &config)
{
    bool passZThresh = false;
    double zDiff = 0.0;
    std::array<double, 3> normal;
    // get zDiff and normal of triangle
    maxZChangeAndNormal(t, delaunay, points, zDiff, normal);
    // get dot product of triangle
    auto prod = std::abs(dotProduct3(normal, config.desiredVector));

    if (config.zThresh > 0 && zDiff < config.zThresh)
    {
        passZThresh = true;
    }

    return prod > config.normThresh || (passZThresh && prod > config.normThreshMin);
}

inline bool validateTriangle4D(size_t t, delaunator::Delaunator &delaunay,  Matrix &points, Config &config)
{
    // hmm simple for right now
    return checkPointClass(t, delaunay, points, config.allowedClass);
}

void __attribute__((optimize("O1"))) createTriHash2(polylidar::unordered_map<size_t, size_t> &triHash, delaunator::Delaunator &delaunay, Matrix &points, Config &config)
{
    // auto points_unchecked = points.unchecked<2>();
    // TODO static_cast<size_t>
    size_t numTriangles = std::floor(delaunay.triangles.size() / 3);
    for (size_t t = 0; t < numTriangles; t++)
    {
        if (validateTriangle2D(t, delaunay, points, config))
        {
            triHash[t] = t;
        }
    }
}

void __attribute__((optimize("O1"))) createTriHash3(polylidar::unordered_map<size_t, size_t> &triHash, delaunator::Delaunator &delaunay, Matrix &points, Config &config)
{
    // auto points_unchecked = points.unchecked<2>();
    // TODO static_cast<size_t>
    size_t numTriangles = std::floor(delaunay.triangles.size() / 3);
    for (size_t t = 0; t < numTriangles; t++)
    {
        bool valid2D = validateTriangle2D(t, delaunay, points, config);
        bool valid3D = validateTriangle3D(t, delaunay, points, config);
        if (valid2D && valid3D)
        {
            triHash[t] = t;
        }
    }
}

void __attribute__((optimize("O1"))) createTriHash4(polylidar::unordered_map<size_t, size_t> &triHash, delaunator::Delaunator &delaunay, Matrix &points, Config &config)
{
    // auto points_unchecked = points.unchecked<2>();
    // std::cout << "Delaunay size " << delaunay.coords.size();
    // TODO static_cast<size_t>
    size_t numTriangles = std::floor(delaunay.triangles.size() / 3);
    for (size_t t = 0; t < numTriangles; t++)
    {
        // std::cout<< "Beginning 2D validation: " << t << std::endl;
        bool valid2D = validateTriangle2D(t, delaunay, points, config);
        // std::cout<< "Beginning 3D validation: " << t << std::endl;
        bool valid3D = validateTriangle3D(t, delaunay, points, config);
        // std::cout<< "Beginning 4D validation: " << t << std::endl;
        bool valid4D = validateTriangle4D(t, delaunay, points, config);
        // std::cout << "Valid4D: " << valid4D << std::endl;
        // auto valid4D = true;
        if (valid2D && valid3D && valid4D)
        {
            triHash[t] = t;
        }
    }
}


void constructPointHash(std::vector<size_t> &plane, delaunator::Delaunator &delaunay, Matrix &points,
                        polylidar::unordered_map<size_t, std::vector<size_t>> &pointHash, polylidar::unordered_map<size_t, size_t> &edgeHash,
                        ExtremePoint &xPoint)
{
    auto &triangles = delaunay.triangles;
    auto &halfedges = delaunay.halfedges;

    // all valid triangles
    polylidar::unordered_map<size_t, size_t> triHash;

    // THIS REDUCED THIS FUNCTIONS RUNTIME BY 1/2
    // LESS ALLOCATIONS AND HASH COLLISIONS
    size_t max_triangles = static_cast<size_t>(plane.size());
    triHash.reserve(max_triangles);

    // This does not seem to make much of a difference
    // But it does not hurt it, Perimeter (boundary edges) grows as sqrt of area (triangles)
    size_t nominal_edges = static_cast<size_t>(std::sqrt(max_triangles) * 3);
    pointHash.reserve(nominal_edges);
    edgeHash.reserve(nominal_edges);

    // auto before = std::chrono::high_resolution_clock::now();
    // This for loop takes up about 50% the execution time of constructPointHash
    // create a hash of all triangles in this plane set
    for (auto &&t : plane)
    {
        triHash[t] = t;
    }
    // auto after = std::chrono::high_resolution_clock::now();
    // auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(after - before);
    // std::cout << "ConstructPointHash - Time creating triangle Hash (ms): " << elapsed.count() << std::endl;


    // auto points_unchecked = points.unchecked<2>();

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
            // TODO static_cast<size_t>
            size_t oppT = std::floor(oppHe / 3);
            // check if this triangle (oppT) is on the convex hull or removed
            if (triHash.find(oppT) == triHash.end())
            {
                // Record this edge
                edgeHash[heIndex] = heIndex;
                // get point index of this half edge, this is an edge leaving from this pi
                auto pi = triangles[heIndex];
                trackExtremePoint(pi, points, xPoint, heIndex);
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
                                   delaunator::Delaunator &delaunay,
                                   size_t startEdge, size_t stopPoint,
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
            std::cerr<< "ERROR! Found a broken edge when extracting a concave section (most likely during hole extraction). Possible that delaunator mislabeled an edge as part of the convex hull" << std::endl;
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
            auto newEdge = getHullEdge(workingEdge, nextEdges, delaunay, isHole);
            workingEdge = newEdge;
        }
    }

    return hullSection;
}

std::vector<std::vector<size_t>> extractInteriorHoles(polylidar::unordered_map<size_t, std::vector<size_t>> pointHash,
                                                      polylidar::unordered_map<size_t, size_t> edgeHash,
                                                      delaunator::Delaunator &delaunay)
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
        auto hole = concaveSection(pointHash, edgeHash, delaunay, startEdge, stopPoint, false);
        if (hole.size() > 0){
            allHoles.push_back(hole);
        }
    }

    return allHoles;
}

Polygon extractConcaveHull(std::vector<size_t> &plane, delaunator::Delaunator &delaunay, Matrix &points, Config &config)
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
    constructPointHash(plane, delaunay, points, pointHash, edgeHash, xPoint);
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
    if (nextEdges.size() > 1) {
        // std::cout << "Right extreme point is connected to a hole. Determining correct edge..." << std::endl;
        // std::cout << "xPoint: " << xPoint << std::endl;
        // std::cout << "Plane size: " << plane.size() << std::endl;
        // std::cout << "Point Hash size: " << pointHash.size() << std::endl;
        // std::cout << "Edge Hash size: " << edgeHash.size() << std::endl;
        startingHalfEdge = getHullEdgeStart(polylidar::UP_VECTOR, nextEdges, delaunay, false);
    }
    auto startingPointIndex = xPoint.xr_pi;
    auto stopPoint = startingPointIndex;
    auto shell = concaveSection(pointHash, edgeHash, delaunay, startingHalfEdge, stopPoint, false);
    auto holes = extractInteriorHoles(pointHash, edgeHash, delaunay);

    // std::vector<std::vector<size_t>> holes;
    poly.shell = std::move(shell);
    poly.holes = std::move(holes);
    return poly;
}

std::vector<Polygon> extractConcaveHulls(std::vector<std::vector<size_t>> planes, delaunator::Delaunator &delaunay, Matrix &points, Config &config)
{

    std::vector<Polygon> polygons;
    for (auto &&plane : planes)
    {
        Polygon poly = extractConcaveHull(plane, delaunay, points, config);
        polygons.push_back(poly);
    }
    return polygons;
}

void extractMeshHash(delaunator::Delaunator &delaunay, polylidar::unordered_map<size_t, size_t> &triHash, size_t seedIdx, std::vector<size_t> &candidates)
{
    // Construct queue for triangle neighbor expansion
    std::queue<size_t> queue;
    // Add seed index to queue and erase from hash map
    queue.push(seedIdx);
    triHash.erase(seedIdx);


    // std::vector<size_t> candidates;
    while (!queue.empty())
    {
        auto tri = queue.front();
        queue.pop();
        candidates.push_back(tri);
        // Get all neighbors that are inside our triangle hash map
        // Loop through every edge of this triangle and get adjacent triangle of this edge
        for (size_t i=0; i < 3; ++i)
        {
            auto e = tri * 3 + i;
            auto opposite = delaunay.halfedges[e];
            if (opposite >= 0)
            {
                // convert opposite edge to a triangle
                // TODO static_cast<size_t>
                size_t tn = std::floor(opposite / 3);
                if (triHash.find(tn) != triHash.end())
                {
                    queue.push(tn);
                    triHash.erase(tn);
                }
            }
        }
    }
}

// TODO
bool passPlaneConstraints(std::vector<size_t> planeMesh, delaunator::Delaunator &delaunay, Config &config)
{
    if (planeMesh.size() < config.minTriangles)
    {
        return false;
    }
    return true;
}

std::vector<std::vector<size_t>> extractPlanes(delaunator::Delaunator &delaunay, Matrix &points, Config &config)
{
    std::vector<std::vector<size_t>> planes;
    polylidar::unordered_map<size_t, size_t> triHash;
    // Reserve hash size to hold all possible triangles
    size_t max_triangles = static_cast<size_t>(delaunay.triangles.size() / 3);
    triHash.reserve(max_triangles);

    // std::cout << "reserving " << max_triangles << " triangles in hash" << std::endl;
    // auto before = std::chrono::high_resolution_clock::now();
    if (config.dim == 2)
    {
        createTriHash2(triHash, delaunay, points, config);
    }
    else if (config.dim == 3)
    {
        createTriHash3(triHash, delaunay, points, config);
    }
    else if (config.dim == 4)
    {
        createTriHash4(triHash, delaunay, points, config);
    }


    // auto after = std::chrono::high_resolution_clock::now();
    // double elapsed = std::chrono::duration_cast<std::chrono::microseconds>(after - before).count() * 1e-3;
    // std::cout << "Tri Hash Creation took " << elapsed << " milliseconds" << std::endl;
    while (!triHash.empty())
    {
        planes.emplace_back(); // construct empty vector inside planes
        auto &planeMesh = planes[planes.size() -1]; // retrieve this newly created vector
        auto seedIdx = std::begin(triHash)->first;
        extractMeshHash(delaunay, triHash, seedIdx, planeMesh);
        // Remove plane if it does not pass constraints
        if (!passPlaneConstraints(planeMesh, delaunay, config))
        {
            planes.pop_back();
        }
    }
    return planes;
}


std::tuple<delaunator::Delaunator, std::vector<std::vector<size_t>>, std::vector<Polygon>> _extractPlanesAndPolygons(Matrix &nparray, Config config)
{
    config.dim = nparray.cols;

    auto before = std::chrono::high_resolution_clock::now();
    delaunator::Delaunator delaunay(nparray);
    delaunay.triangulate();
    auto after = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(after - before);
    // std::cout << "Delaunay took " << elapsed.count() << " milliseconds" << std::endl;

    before = std::chrono::high_resolution_clock::now();
    std::vector<std::vector<size_t>> planes = extractPlanes(delaunay, nparray, config);
    after = std::chrono::high_resolution_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(after - before);
    // std::cout << "Plane Extraction took " << elapsed.count() << " milliseconds" << std::endl;

    before = std::chrono::high_resolution_clock::now();
    std::vector<Polygon> polygons = extractConcaveHulls(planes, delaunay, nparray, config);
    after = std::chrono::high_resolution_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(after - before);
    // std::cout << "Polygon Hull Extraction took " << elapsed.count() << " milliseconds" << std::endl;
    return std::make_tuple(delaunay, planes, polygons);

    // nparray2D is a contigious buffer of (ROWS,2)
}

std::vector<Polygon> _extractPolygons(Matrix &nparray, Config config)
{
    config.dim = nparray.cols;

    auto before = std::chrono::high_resolution_clock::now();
    delaunator::Delaunator delaunay(nparray);
    delaunay.triangulate();
    auto after = std::chrono::high_resolution_clock::now();
    float elapsed_d = std::chrono::duration_cast<std::chrono::microseconds>(after - before).count() * 1e-3;
    // std::cout << "Delaunay took " << elapsed_d << " milliseconds" << std::endl;

    before = std::chrono::high_resolution_clock::now();
    std::vector<std::vector<size_t>> planes = extractPlanes(delaunay, nparray, config);
    after = std::chrono::high_resolution_clock::now();
    float elapsed_ep = std::chrono::duration_cast<std::chrono::microseconds>(after - before).count() * 1e-3;
    // std::cout << "Plane Extraction took " << elapsed_ep << " milliseconds" << std::endl;

    before = std::chrono::high_resolution_clock::now();
    std::vector<Polygon> polygons = extractConcaveHulls(planes, delaunay, nparray, config);
    after = std::chrono::high_resolution_clock::now();
    float elapsed_ch = std::chrono::duration_cast<std::chrono::microseconds>(after - before).count() * 1e-3;
    // std::cout << "Polygon Hull Extraction took " << elapsed_ch << " milliseconds" << std::endl;
    return polygons;
}

std::vector<Polygon> _extractPolygonsAndTimings(Matrix &nparray, Config config, std::vector<float> &timings)
{
    config.dim = nparray.cols;

    auto before = std::chrono::high_resolution_clock::now();
    delaunator::Delaunator delaunay(nparray);
    delaunay.triangulate();
    auto after = std::chrono::high_resolution_clock::now();
    float elapsed_d = std::chrono::duration_cast<std::chrono::microseconds>(after - before).count() * 1e-3;
    // std::cout << "Delaunay took " << elapsed_d << " milliseconds" << std::endl;

    before = std::chrono::high_resolution_clock::now();
    std::vector<std::vector<size_t>> planes = extractPlanes(delaunay, nparray, config);
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


} // namespace polylidar