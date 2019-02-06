# Polylidar V2



## Installing

### Pymultiastar

1. Install conda
2. `conda install -c conda-forge pybind11`
3. `python setup.py build install`


## API

export function extractPlanesAndPolygons(
  points:number[][],
  config: IConfig2D | IConfig3D,
  pointsAll?: any
): [Delaunator<number>, number[][], IPolygon[]] 

export function extractConcaveHulls(
  planes: number[][],
  delaunay: Delaunator<number>,
  config: IConfig2D | IConfig3D,
): IPolygon[]

export function extractConcaveHull(
  plane: number[],
  delaunay: Delaunator<number>,
  _config?: IConfig2D | IConfig2D,
): IPolygon 

function extractInteriorHoles(
  pointHash: ArrayHash,
  edgeHash: NumberHash,
  delaunay: Delaunator<number>,
)

function concaveSection(
  pointHash: ArrayHash,
  edgeHash: NumberHash,
  delaunay: Delaunator<number>,
  startEdge: number,
  stopPoint: number,
  isHole: boolean = false
)

function constructPointHash(
  plane: number[],
  delaunay: Delaunator<number>,
): [ArrayHash, NumberHash, NumberHash, IExtremePoint]

function maxPoint(
  pi: number,
  delaunay: Delaunator<number>,
  extremePoint: IExtremePoint,
  he: number,
)

export function extractPlanes(
  delaunay: Delaunator<number>,
  config: IConfig2D | IConfig2D,
  pointsAll?


export function create_planes(
  delaunay: Delaunator<number>,
  config: IConfig2D | IConfig3D,
  triValidator: IValidateTriangle,
  pointsAll: any

function extractMeshHash(
  delaunay: Delaunator<number>,
  triHash: TriHash,
  seedIdx: number,

OVER LOADED FUNCTION based on TYPE
  function createTriangleHash(
  delaunay: Delaunator<number>,
  triValidator: IValidateTriangle,
  pointsAll: any, <-- This type


Idea

export function create_planes(
  delaunay: Delaunator<number>,
  config: IConfig2D | IConfig3D,
  triValidator: IValidateTriangle,
  pointsAll: any,
) {
  // Create hash map
  // const numTriangles = Math.floor(delaunay.triangles.length / 3)
  // Check points all shape
  based upon the shape call either createTriHash2, 3, or 4
  2 will call 2,, 4 will call 3
  // config will **always** have defaults for all 4 dimensions
  const triHash = createTriangleHash(delaunay, triValidator, pointsAll)

  let triKeys = []
  const allPlanes: number[][] = []
  while (true) {
    triKeys = Object.keys(triHash)
    if (triKeys.length < 1) {
      break
    }
    const seedIdx = parseFloat(triKeys[0])
    // const seedIdx = triKeys[0]
    const planePatch = extractMeshHash(delaunay, triHash, seedIdx)
    if (applyPlaneConstraints(planePatch, config, delaunay)) {
      allPlanes.push(planePatch)
    }
  }

  return allPlanes
}

## Multi-Goal Path Planning


## Benchmark



