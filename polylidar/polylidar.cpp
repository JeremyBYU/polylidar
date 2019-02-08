#include "polylidar.hpp"

double DESIRED_VECTOR[3] = {0.0, 0.0, 1.0};
namespace py = pybind11;
using namespace pybind11::literals;


namespace polylidar {

    // Polygon::set()

    std::ostream& operator<<(std::ostream& os, const Config& config)
    {   
        os << "Dim=" << config.dim << " alpha=" << config.alpha << " xyThres=" << config.xyThresh << " minTriangles=" << config.minTriangles 
        << " minBboxArea=" << config.minBboxArea << " zThresh=" << config.zThresh << " normThresh=" << config.normThresh
        << " allowedClass=" << config.allowedClass 
        << " desiredVector= [" << (*config.desiredVector)[0] << ", " << (*config.desiredVector)[1] << ", " << (*config.desiredVector)[2] << "]";

        return os;
    }

    std::ostream& operator<<(std::ostream& os, const std::vector<size_t>& values)
    {   
        for (auto &&val : values) {
            os << val << ", ";
        }

        return os;
    }

    std::ostream& operator<<(std::ostream& os, const ExtremePoint &values)
    {   
        os << "xr_he" << values.xr_he << " xr_pi" << values.xr_pi << " xr_val" << values.xr_val;

        return os;
    }

    void copy2Ddata(py::array_t<double> &src, py::array_t<double> &dest) {
        auto shape = src.shape();
        auto rows = shape[0];

        auto src_ = src.unchecked<2>();
        auto dest_ = dest.mutable_unchecked<2>();
        for(int i =0; i< rows; i++) {
            dest_(i, 0) = src_(i, 0);
            dest_(i, 1) = src_(i, 1);
        }
    }

    inline bool validateTriangle2D(size_t t, delaunator::Delaunator &delaunay, pybind11::detail::unchecked_reference<double, 2L> points, Config &config) {
        // auto maxXY = getMaxDimTriangle(t, delaunay, points);
        // std::cout << "Triangle " << t << " Radius: " << radius << std::endl;
        if (config.alpha > 0.0 && circumsribedRadius(t, delaunay, points) > 1.0 / config.alpha) {
            return false;
        }
        if (config.xyThresh > 0.0 && getMaxDimTriangle(t, delaunay, points) > config.xyThresh) {
            return false;
        }
        return true;
    }

    inline bool validateTriangle3D(size_t t, delaunator::Delaunator &delaunay, pybind11::detail::unchecked_reference<double, 2L> points, Config &config) {
        bool passZThresh = false;
        double zDiff = 0.0;
        std::array<double, 3> normal;
        maxZChangeAndNormal(t, delaunay, points, zDiff, normal);
        if (config.zThresh > 0 && zDiff < config.zThresh) {
            passZThresh = true;
        }

        auto test = config.desiredVector;
        auto prod = std::abs(dotProduct3(normal, *config.desiredVector));
        if (prod < config.normThresh && !passZThresh) {
            return false;
        }
        return true;

    }

    inline bool validateTriangle4D(size_t t, delaunator::Delaunator &delaunay, pybind11::detail::unchecked_reference<double, 2L> points, Config &config) {
        // hmm simple for right now
        return checkPointClass(t, delaunay, points, config.allowedClass);
    }


    void createTriHash2(std::unordered_map<size_t, size_t> &triHash, delaunator::Delaunator &delaunay, py::array_t<double> &points, Config &config ) {
        auto points_unchecked = points.unchecked<2>();
        size_t numTriangles = std::floor(delaunay.triangles.size() / 3 );
        for (size_t t = 0; t < numTriangles; t++) {
            if(validateTriangle2D(t, delaunay, points_unchecked, config)) {
                triHash[t] = t;
            }
        }
    }

    void createTriHash3(std::unordered_map<size_t, size_t> &triHash, delaunator::Delaunator &delaunay, py::array_t<double> &points, Config &config ) {
        auto points_unchecked = points.unchecked<2>();
        size_t numTriangles = std::floor(delaunay.triangles.size() / 3 );
        for (size_t t = 0; t < numTriangles; t++) {
            bool valid2D = validateTriangle2D(t, delaunay, points_unchecked, config);
            bool valid3D = validateTriangle3D(t, delaunay, points_unchecked, config);
            if(valid2D && valid3D) {
                triHash[t] = t;
            }
        }
    }

    void createTriHash4(std::unordered_map<size_t, size_t> &triHash, delaunator::Delaunator &delaunay, py::array_t<double> &points, Config &config ) {
        auto points_unchecked = points.unchecked<2>();
        // std::cout << "Delaunay size " << delaunay.coords.size();
        size_t numTriangles = std::floor(delaunay.triangles.size() / 3 );
        for (size_t t = 0; t < numTriangles; t++) {
            // std::cout<< "Beginning 2D validation: " << t << std::endl;
            bool valid2D = validateTriangle2D(t, delaunay, points_unchecked, config);
            // std::cout<< "Beginning 3D validation: " << t << std::endl;
            bool valid3D = validateTriangle3D(t, delaunay, points_unchecked, config);
            // std::cout<< "Beginning 4D validation: " << t << std::endl;
            bool valid4D = validateTriangle4D(t, delaunay, points_unchecked, config);
            // std::cout << "Valid4D: " << valid4D << std::endl;
            // auto valid4D = true;
            if(valid2D && valid3D && valid4D) {
                triHash[t] = t;
            }
        }
    }

    std::vector<size_t> trianglesAdjacentToTriangle(size_t t, delaunator::Delaunator &delaunay, std::unordered_map<size_t, size_t> &triHash) {
        std::vector<size_t> triangles;
        std::vector<size_t> edgesOfTriangle = {t * 3, t * 3 + 1, t * 3 + 2};
        for (auto &&e : edgesOfTriangle)
        {
            auto opposite = delaunay.halfedges[e];
            if(opposite >= 0){
                // convert opposite edge to a triangle
                auto tn = std::floor(opposite / 3);
                if (triHash.count(tn) > 0) {
                    triangles.push_back(tn);
                }
            }
        }
        return triangles;
    }


    std::tuple<std::unordered_map<size_t, std::vector<size_t>>, std::unordered_map<size_t, size_t>, ExtremePoint> constructPointHash(std::vector<size_t> plane, delaunator::Delaunator &delaunay, py::array_t<double> &points) {
        auto &triangles = delaunay.triangles;
        auto &halfedges = delaunay.halfedges;
        // all incoming half eddges to a point index
        std::unordered_map<size_t, std::vector<size_t>> pointHash;
        // all valid triangles
        std::unordered_map<size_t, size_t> triHash;
        // all empty border half edges
        std::unordered_map<size_t, size_t> edgeHash;
        // create a hash of all triangles in this plane set
        for (auto &&t : plane) {
            triHash[t] = t;
        }

        // extreme point
        ExtremePoint xPoint;
        auto points_unchecked = points.unchecked<2>();


        // Loop through every triangle in the plane
        for (auto &&t : plane) {
            // Loop through every edge in the triangle
            for (int i = 0; i < 3; i++) {
                // get halfedge index
                auto heIndex = t * 3 + i;
                // get the adjacent edge of this triangle edge
                auto oppHe = halfedges[heIndex];
                auto oppT = std::floor(oppHe / 3);
                // check if this triangle (oppT) is on the convex hull or removed
                if (triHash.count(oppT) == 0) {
                    // Record this edge
                    edgeHash[heIndex] = heIndex;
                    // get point index of this half edge, this is an edge leaving from this pi
                    auto pi = triangles[heIndex];
                    trackExtremePoint(pi, points_unchecked, xPoint, heIndex);
                    // Check if the point has already been indexed
                    if (pointHash.count(pi) == 0) {
                        // construct a new vector holding this half edge index
                        pointHash[pi] = std::vector<size_t>(1, heIndex);
                    } else {
                        // point already exists, just append to it
                        pointHash[pi].push_back(heIndex);
                    }

                }
            }
        }

        return std::make_tuple(std::move(pointHash), std::move(edgeHash), std::move(xPoint));
    }


    std::vector<size_t> concaveSection(std::unordered_map<size_t, std::vector<size_t>> &pointHash, 
                        std::unordered_map<size_t, size_t> &edgeHash,
                        delaunator::Delaunator &delaunay,
                        size_t startEdge, size_t stopPoint,
                        bool isHole) {

        // std::cout << "Inside concave section" <<std::endl;
        std::vector<size_t> hullSection;

        auto &triangles = delaunay.triangles;
        auto &coords = delaunay.coords;
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
            if (nextPi == stopPoint) {
                return hullSection;
            }

            // Get outgoing edges for this point
            auto &nextEdges = pointHash[nextPi];

            // std::cout<< "nextEdges: " << nextEdges << std::endl;
            
            // filter edges that have already been seen!
            nextEdges.erase(std::remove_if(nextEdges.begin(), nextEdges.end(),
                            [&edgeHash](size_t &e){ return edgeHash.count(e) == 0;}),
                            nextEdges.end());
            // std::cout<< "nextEdges after filter: " << nextEdges << std::endl;
            if (nextEdges.size() == 1) {
                workingEdge = nextEdges[0];
            } else {
                // We have a junction of outgoing edges at this point
                auto newEdge = getHullEdge(workingEdge, nextEdges, delaunay, isHole);
                workingEdge = newEdge;
            }
            // std::cout<< "Next working edge: " << workingEdge << std::endl;
            // std::vector<size_t> nextEdgesFiltered;
            // std::remove_copy (nextEdges.begin(), nextEdges.end(),nextEdgesFiltered.begin(),20)
        
        }

        return hullSection;
    }

    std::vector<std::vector<size_t>> extractInteriorHoles(std::unordered_map<size_t, std::vector<size_t>> pointHash,
                                                        std::unordered_map<size_t, size_t> edgeHash,
                                                        delaunator::Delaunator &delaunay)
    {
        std::vector<std::vector<size_t>>  allHoles;
        auto &triangles = delaunay.triangles;
        while (true)
        {
            if(edgeHash.empty()) {
                break;
            }
            auto startEdge = std::begin(edgeHash)->first;
            // auto startingPointIndex = triangles[startEdge];
            auto stopPoint = triangles[startEdge];
            auto hole = concaveSection(pointHash, edgeHash, delaunay, startEdge, stopPoint, false);
            allHoles.push_back(hole);

        }

        return allHoles;
    }

    Polygon extractConcaveHull(std::vector<size_t> plane, delaunator::Delaunator &delaunay, py::array_t<double> &points, Config &config) {
        Polygon poly;
        // point hash
        std::unordered_map<size_t, std::vector<size_t>> pointHash;
        // hash of all empty border half edges
        std::unordered_map<size_t, size_t> edgeHash;
        // the left and right most extreme points
        ExtremePoint xPoint;
        std::tie(pointHash, edgeHash, xPoint) = constructPointHash(plane, delaunay, points);

        // std::cout << "xPoint: " << xPoint << std::endl;
        // std::cout << "Plane size: " << plane.size() << std::endl;
        // std::cout << "Point Hash size: " << pointHash.size() << std::endl;
        // std::cout << "Edge Hash size: " << edgeHash.size() << std::endl;
        
        auto startingHalfEdge = xPoint.xr_he;
        auto startingPointIndex = xPoint.xr_pi;
        auto stopPoint = startingPointIndex;
        auto shell = concaveSection(pointHash, edgeHash, delaunay, startingHalfEdge, stopPoint, false);
        auto holes = extractInteriorHoles(pointHash, edgeHash, delaunay);

        // std::vector<std::vector<size_t>> holes;
        poly.shell = std::move(shell); // TODO these are copies, use std::move?
        poly.holes = std::move(holes);
        return poly;
    }

    std::vector<Polygon> extractConcaveHulls(std::vector<std::vector<size_t>> planes, delaunator::Delaunator &delaunay, py::array_t<double> &points, Config &config) {
        
        std::vector<Polygon> polygons;
        for(auto &&plane: planes) {
            Polygon poly = extractConcaveHull(plane, delaunay, points, config);
            polygons.push_back(poly);
        }
        return polygons;
    }


    std::vector<size_t> extractMeshHash(delaunator::Delaunator &delaunay, std::unordered_map<size_t, size_t> &triHash, size_t seedIdx) {
        // Construct queue for triangle neighbor expansion
        std::queue<size_t> queue;
        // Add seed index to queue and erase from hash map
        queue.push(seedIdx);
        triHash.erase(seedIdx);

        std::vector<size_t> candidates;
        while(!queue.empty()) {
            auto tri = queue.front();
            queue.pop();
            candidates.push_back(tri);
            // Get all neighbors that are inside our triangle hash map
            auto neighbors = trianglesAdjacentToTriangle(tri, delaunay, triHash);
            for (auto && t: neighbors) {
                queue.push(t);
                triHash.erase(t);
            } 

        }
        return candidates;
    }

    // TODO
    bool passPlaneConstraints(std::vector<size_t> planeMesh, delaunator::Delaunator &delaunay, Config &config) {
        if(planeMesh.size() < config.minTriangles) {
            return false;
        }
        return true;
    }

    std::vector<std::vector<size_t>> extractPlanes(delaunator::Delaunator &delaunay, py::array_t<double> &points, Config &config) {
        std::vector<std::vector<size_t>> planes;
        std::unordered_map<size_t, size_t> triHash;
        
        if (config.dim == 2) {
            createTriHash2(triHash, delaunay, points, config);
        }
        if (config.dim == 3) {
            createTriHash3(triHash, delaunay, points, config);
        }
        if (config.dim == 4) {
            createTriHash4(triHash, delaunay, points, config);
        }

        while(!triHash.empty()) {
            auto seedIdx = std::begin(triHash)->first;
            std::vector<size_t> planeMesh = extractMeshHash(delaunay, triHash, seedIdx);
            if (passPlaneConstraints(planeMesh, delaunay, config)) {
                planes.emplace_back(planeMesh);
            }
            
        }
        return planes;
    }

    std::tuple<delaunator::Delaunator, std::vector<std::vector<size_t>>, std::vector<Polygon>> _extractPlanesAndPolygons(py::array_t<double> nparray, Config config) {
        auto shape = nparray.shape();
        int rows = shape[0];
        int cols = shape[1];
        config.dim = cols;

        // std::cout << "Config: " << config <<std::endl;

        // std::cout << "Shape " << rows << "," << cols << std::endl;
        py::array_t<double> temp;
        py::array_t<double> *nparray2D;
        nparray2D = &nparray;
        if (cols > 2) {
            temp.resize({rows, 2});
            copy2Ddata(nparray, temp);
            nparray2D = &temp;
        }
        auto before = std::chrono::high_resolution_clock::now();
        delaunator::Delaunator delaunay(*nparray2D);
        delaunay.triangulate();
        auto after = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(after - before);
        std::cout << "Delaunay took " << elapsed.count() << " milliseconds" << std::endl;
        // std::vector<Polygon> polygonsTest;
        // Polygon testPoly;
        // testPoly.shell = {1, 2, 3};
        // polygonsTest.push_back(testPoly);
        
        before = std::chrono::high_resolution_clock::now();
        std::vector<std::vector<size_t>> planes = extractPlanes(delaunay, nparray, config);
        after = std::chrono::high_resolution_clock::now();
        elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(after - before);
        std::cout << "Plane Extraction took " << elapsed.count() << " milliseconds" << std::endl;

        before = std::chrono::high_resolution_clock::now();
        std::vector<Polygon> polygons = extractConcaveHulls(planes, delaunay, nparray, config);
        after = std::chrono::high_resolution_clock::now();
        elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(after - before);
        std::cout << "Polygon Hull Extraction took " << elapsed.count() << " milliseconds" << std::endl;
        return std::make_tuple(delaunay, planes, polygons);



        // nparray2D is a contigious buffer of (ROWS,2)
    }


    std::tuple<delaunator::Delaunator, std::vector<std::vector<size_t>>, std::vector<Polygon>> extractPlanesAndPolygons(py::array_t<double> nparray, int dim = DEFAULT_DIM,
                                  double alpha = DEFAULT_ALPHA, double xyThresh = DEFAULT_XYTHRESH, size_t minTriangles = DEFAULT_MINTRIANGLES,
                                  double minBboxArea = DEFAULT_MINBBOX, double zThresh = DEFAULT_ZTHRESH, 
                                  double normThresh = DEFAULT_NORMTHRESH, double allowedClass = DEFAULT_ALLOWEDCLASS)
    {
        // This function allows us to convert keyword arguments into a configuration struct
        Config config {dim, alpha, xyThresh, minTriangles, minBboxArea, zThresh, normThresh, allowedClass};

        

        return _extractPlanesAndPolygons(nparray, config);


        
    }
                    
    
  



}