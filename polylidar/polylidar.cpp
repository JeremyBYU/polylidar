#include "polylidar.hpp"

double DESIRED_VECTOR[3] = {0.0, 0.0, 1.0};
namespace py = pybind11;
using namespace pybind11::literals;


namespace polylidar {

    std::ostream& operator<<(std::ostream& os, const Config& config)
    {   
        os << "Config: Dim=" << config.dim << " alpha=" << config.alpha << " minTriangles=" << config.minTriangles 
        << " minBboxArea=" << config.minBboxArea << " zThresh=" << config.zThresh << " normThresh=" << config.normThresh
        << " allowedClass=" << config.allowedClass;

        return os;
    }

    // void printConfig(const Config& config) {
    //     std::cout << "Config: Dim=" << config.dim << " alpha=" << config.alpha << " minTriangles=" << config.minTriangles 
    //     << " minBboxArea=" << config.minBboxArea << " zThresh=" << config.zThresh << " normThresh=" << config.normThresh
    //     << " allowedClass=" << config.allowedClass << std::endl;
    // }

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
        auto radius = circumsribedRadius(t, delaunay, points);
        // std::cout << "Triangle " << t << " Radius: " << radius << std::endl;
        if (radius > 1.0 / config.alpha) {
            return false;
        }
        return true;
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

    Polygon extractConcaveHull(std::vector<size_t> plane, delaunator::Delaunator &delaunay, Config &config) {

    }

    std::vector<Polygon> extractConcaveHulls(std::vector<std::vector<size_t>> planes, delaunator::Delaunator &delaunay, Config &config) {
        
        std::vector<Polygon> polygons;
        for(auto &&plane: planes) {
            Polygon poly = extractConcaveHull(plane, delaunay, config);
        }
        return polygons;
    }

    // export function extractConcaveHulls(
    // planes: number[][],
    // delaunay: Delaunator<number>,
    // config: IConfig2D | IConfig3D,
    // ): IPolygon[] {
    // // Each plane will be a polygon with possible holes in it
    // const concaveHulls: IPolygon[] = []
    // for (const plane of planes) {
    //     const cHull = extractConcaveHull(plane, delaunay, config)
    //     concaveHulls.push(cHull)
    // }

    // return concaveHulls
    // }

//     export function trianglesAdjacentToTriangle(
//   delaunay: Delaunator<number>,
//   t: number,
// ): number[] {
//   const adjacentTriangles = []
//   for (const e of edgesOfTriangle(t)) {
//     const opposite = delaunay.halfedges[e]
//     if (opposite >= 0) {
//       adjacentTriangles.push(triangleOfEdge(opposite))
//     }
//   }

//   return adjacentTriangles
// }

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
        return true;
    }

    std::vector<std::vector<size_t>> extractPlanes(delaunator::Delaunator &delaunay, py::array_t<double> &points, Config &config) {
        std::vector<std::vector<size_t>> planes;
        std::unordered_map<size_t, size_t> triHash;
        
        if (config.dim == 2) {
            createTriHash2(triHash, delaunay, points, config);
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

        // std::cout << "Shape " << rows << "," << cols << std::endl;
        py::array_t<double> temp;
        py::array_t<double> *nparray2D;
        nparray2D = &nparray;
        if (cols > 2) {
            temp.resize({rows, 2});
            copy2Ddata(nparray, temp);
            nparray2D = &temp;
        }

        delaunator::Delaunator delaunay(*nparray2D);
        delaunay.triangulate();


        std::vector<Polygon> polygons;
        std::vector<std::vector<size_t>> planes = extractPlanes(delaunay, nparray, config);
        return std::make_tuple(delaunay, planes, polygons);



        // nparray2D is a contigious buffer of (ROWS,2)
    }


    std::tuple<delaunator::Delaunator, std::vector<std::vector<size_t>>, std::vector<Polygon>> extractPlanesAndPolygons(py::array_t<double> nparray, int dim = DEFAULT_DIM,
                                  double alpha = DEFAULT_ALPHA, size_t minTriangles = DEFAULT_MINTRIANGLES,
                                  double minBboxArea = DEFAULT_MINBBOX, double zThresh = DEFAULT_ZTHRESH, 
                                  double normThresh = DEFAULT_NORMTHRESH, double allowedClass = DEFAULT_ALLOWEDCLASS)
    {
        // This function allows us to convert keyword arguments into a configuration struct
        Config config {dim, alpha, minTriangles, minBboxArea, zThresh, normThresh, allowedClass};

        std::cout << "Config: " << config <<std::endl;

        return _extractPlanesAndPolygons(nparray, config);


        
    }
                    
    
  



}