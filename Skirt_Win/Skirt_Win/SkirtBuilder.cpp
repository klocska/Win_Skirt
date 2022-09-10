// Copyright Maxim Prut klocska@gmail.com, 2022.

// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
// THIS SOFTWARE.

#include "SkirtBuilder.h"

#include "Mesh/Mesh.h"
#include "ThirdPaty/STLIMport.h"

//#pragma optimize("", off)

// angles that less than this threshold are processed by single step
constexpr double AtomicAngle = 10. * PI / 180;
const double NoExposeThreshold = std::cos(AtomicAngle);

////////////////////////////////////////////////////////////////////////////////
void SkirtBuilder::buildGeneratrixTop(const Math_NS::Vector3P& rootPoint, 
  const Math_NS::Vector3P& vertNormal, 
  const Math_NS::Vector3P& planeDir,
  Pack& pack)
{
  Generatrix newGeneratrix(rootPoint, vertNormal, planeDir);

  if (newGeneratrix.buildTopPart(d_curData.upperSmoothR, d_curData.slopeAngle, d_curData.angularSegments))
  {
    d_minZ = std::min(d_minZ, d_minZ);
    pack.push_back(std::move(newGeneratrix));
  }
}

////////////////////////////////////////////////////////////////////////////////
void SkirtBuilder::buildGenBottom(Pack& pack, double packShift)
{
  for (auto& gen : pack)
  {
    gen.buildBottomPart(d_curData.lowerSmoothR, d_curData.slopeAngle, d_curData.straightSegments, d_curData.angularSegments, packShift);
  }

}

////////////////////////////////////////////////////////////////////////////////
SkirtBuilder::NormAndDir SkirtBuilder::evaluateNormAndDir(const Math_NS::Vector3P& vertNormal,
  const Math_NS::Vector3P& edgeChord, const std::optional<Math_NS::Vector3P>& knownDir )
{
  NormAndDir result;
  
  result.direction = !knownDir ? (edgeChord ^ vertNormal).makeUnit() : *knownDir;

  Math_NS::Vector3P horEdgeNorm(result.direction.x, result.direction.y, 0.);
  horEdgeNorm = horEdgeNorm.makeUnit();

  Math_NS::Vector3P genPlaneNorm = horEdgeNorm ^ Math_NS::Vector3D::zAx();
  auto mult = vertNormal * genPlaneNorm;
  Math_NS::Vector3P correction = genPlaneNorm * mult;
  result.normal = (vertNormal - correction).makeUnit();

  return result;
}


////////////////////////////////////////////////////////////////////////////////
SkirtBuilder::PlaneParams SkirtBuilder::evaluatePlane(CMesh_NS::EdgeID edgeID, CMesh_NS::EdgeID nextEdgeID)
{
  SkirtBuilder::PlaneParams params;

  CMesh_NS::VertID rootVertex = d_sourceMesh.getCoEdge(edgeID).getV2();
  params.startPoint = d_sourceMesh.getPoint(rootVertex);

  // generatrix plane is a vertical plane that is defined by (almost horizontal) normal to edge and zAxis

  Math_NS::Vector3P vertexNormal = d_sourceMesh.evaluateMeanNormal(rootVertex);
  params.vertexNormal = vertexNormal;

  Math_NS::Vector3P edgeChord = d_sourceMesh.findEdgeDir(edgeID);

  NormAndDir startND = evaluateNormAndDir(vertexNormal, edgeChord, std::nullopt);
  params.startDirOnPlane = startND.direction;
  params.startVertNormal = startND.normal;

  Math_NS::Vector3P nextEdgeChord = d_sourceMesh.findEdgeDir(nextEdgeID);
  NormAndDir endND = evaluateNormAndDir(vertexNormal, nextEdgeChord, std::nullopt);
  params.nextDirOnPlane = endND.direction;
  params.nextVertNormal = endND.normal;

  NormAndDir intermediateND = evaluateNormAndDir(vertexNormal, (edgeChord + nextEdgeChord).makeUnit(), std::nullopt);
  params.intermediateDirOnPlane = intermediateND.direction;
  params.intermediateVertNormal = intermediateND.normal;

  return params;
}

////////////////////////////////////////////////////////////////////////////////
CMesh_NS::CMesh SkirtBuilder::renderPackToMesh(const std::vector<Pack>& packSet, const Math_NS::TransMat4P& transform) const
{
  CMesh_NS::CMesh newMesh(d_sourceMesh);
  newMesh = newMesh.applyTransform(transform);

  for (const auto& pack : packSet)
  {

    const int packSize = isize(pack);
    if (packSize < 2)
      return newMesh;

    for (int i = 0; i < packSize; ++i)
    {
      int nextInd = (i + 1 < packSize) ? i + 1 : 0;

      int genSize = isize(pack[i].getPoints());
      for (int j = 0; j + 1 < genSize; ++j)
      {
        newMesh.addTrianlge(pack[nextInd].getPoints()[j], pack[i].getPoints()[j], pack[i].getPoints()[j + 1]);
        newMesh.addTrianlge(pack[i].getPoints()[j + 1], pack[nextInd].getPoints()[j + 1], pack[nextInd].getPoints()[j]);
      }
    }
  }

  return newMesh;
}

////////////////////////////////////////////////////////////////////////////////
void SkirtBuilder::expandDirections(const PlaneParams& params,   Pack& pack)
{
  buildGeneratrixTop(params.startPoint, params.startVertNormal, params.startDirOnPlane, pack);

  Math_NS::Vector3P normToPlane = params.startDirOnPlane ^ params.nextDirOnPlane;
  const double angle = std::asin(normToPlane.lengh());
  normToPlane = normToPlane.makeUnit();

  const Math_NS::Vector3P basisVector1 = params.startDirOnPlane;
  const Math_NS::Vector3P basisVector2 = normToPlane ^ params.startDirOnPlane;

  const int steps = int(std::ceil( angle / AtomicAngle)) - 1;

  if (steps != 0) // I'd use an assert here but I haven't implemented it yet
  {
    const double angleStep = angle / steps;

    for (int i = 1; i < steps; ++i)
    {
      const Math_NS::Vector3P curPlaneDir = basisVector1 * std::cos(i * angleStep) + basisVector2 * std::sin(i * angleStep);
      NormAndDir curND = evaluateNormAndDir(params.vertexNormal, Math_NS::Vector3P::zero(), curPlaneDir);
      buildGeneratrixTop(params.startPoint, curND.normal, curND.direction, pack);
    }
  }
  buildGeneratrixTop(params.startPoint, params.nextVertNormal, params.nextDirOnPlane, pack);
}

////////////////////////////////////////////////////////////////////////////////
SkirtBuilder::Report SkirtBuilder::run(const SkirtBuilder::Data& data)
{
  d_curData = data;
  Report report;
  
  const auto& orphanList = d_sourceMesh.getOrphanList();

  std::set<CMesh_NS::EdgeID> affectedSet;

  CMesh_NS::EdgeID startEdge = *orphanList.begin();
  CMesh_NS::EdgeID curEdge = startEdge;
  
  std::vector<Pack> result;
  std::set<CMesh_NS::FaceID> orphanFaces;

  double minZ = std::numeric_limits<double>::max();

  for (const auto& point : d_sourceMesh.getAllPoints())
    minZ = std::min(static_cast<double>(point.z), minZ);
    
  // generate objects and top roundings for packs
  do 
  {
    Pack pack;
    do
    {
      orphanFaces.emplace(d_sourceMesh.getCoEdge(curEdge).getFace());
      auto nextOrphanOpt = d_sourceMesh.findNextOrphan(curEdge);
      if (!nextOrphanOpt) // cannot find next edge in chain. Now it's checked before the run call
      {
        report.status = ErrorCode::BAD_MESH;
        return report;
      }

      CMesh_NS::EdgeID nextOrphan = *nextOrphanOpt;
      auto stepParams = evaluatePlane(curEdge, nextOrphan);
      
      // straight or concave case - one generatrix (here we have a danger of degeneracies in case when we have a number small concave edges)
      // it can be solve only via more complicated operations boolean-like type
      if ((stepParams.nextDirOnPlane ^ stepParams.startDirOnPlane) * Math_NS::Vector3P::zAx() < 0 && 
        abs(stepParams.nextDirOnPlane * stepParams.startDirOnPlane) < NoExposeThreshold )
      {
        // convex case. Expand corner by a family of generatrix
        expandDirections(stepParams, pack);
      }
      else 
      {
        // build single generatrix as average between directions of neighbour edges
        buildGeneratrixTop(stepParams.startPoint, stepParams.intermediateVertNormal, stepParams.intermediateDirOnPlane, pack);
      }
       
      affectedSet.emplace(curEdge);
      curEdge = nextOrphan;

    } while (affectedSet.find(curEdge) == affectedSet.end()); // repeat while the loop isn't closed

    result.push_back(std::move(pack));
  } while ( isize(affectedSet) != isize(orphanList) ); // repeat while we have not visited edges. Yeah, it may work for holes as well.


  // find compensation by z shift. The sole is supposed to touch XY plane and height shouldn't be less than minimalHeight in params
  // find he lowest z for current pacj first to be able to evaluate straight parts
  const double lowerZ = findLowestInPacks(result);

  const double deltaZ = minZ - lowerZ;  // z distance for top roundings

  // z distance for bottom roundings (they are the same since are contacting with similar slopess)
  const double bottomRoundingHeight = d_curData.lowerSmoothR * (1.0 - std::cos(d_curData.slopeAngle));
  double shift = bottomRoundingHeight - lowerZ + 0.01; // 0.01 is a technological shift to avoid degeneracies (there is no special treatment for them)

  // the height must not be less than minimalHeight BUT must allow enough space for roundings
  if ((shift + deltaZ) < data.minimalHeight)
    shift += (data.minimalHeight - deltaZ - bottomRoundingHeight);
  
  Math_NS::TransMat4P shiftMatrix(Math_NS::Vector3P{ 0.,0.,shift });

  // apply shift and build sole parts
  for (auto& pack : result)
    buildGenBottom(pack, shift);
     
  // render generatrix to poligons and add to mesh
  report.result = renderPackToMesh(result, shiftMatrix);
  return report;
}

////////////////////////////////////////////////////////////////////////////////
double SkirtBuilder::findLowestInPacks(const std::vector<Pack>& packSet)
{
  Math_NS::Vector3P lowestPoint;
  double z = std::numeric_limits<double>::max();

  for (const auto& pack : packSet)
  {
    for( const auto& gen: pack)
    z = std::min(z, gen.getLowerZ());
  }

  return z;
}

////////////////////////////////////////////////////////////////////////////////
bool Generatrix::buildTopPart(double radii, double slope, int nPoints)
{
  // start slope is evaluated as a angle between the normal and z axis.
  auto actualSlopeCos = d_vertNormal * Math_NS::Vector3P::zAx();
  auto actualSlopeAngle = std::acos(actualSlopeCos);

  Math_NS::Vector3P planeDirInHor(d_planeDirection.x, d_planeDirection.y, 0.);
  auto backInclinationFlg = d_vertNormal * planeDirInHor < 0.;

  if (backInclinationFlg)
    actualSlopeAngle = -actualSlopeAngle;
  
  // isConvex means that the requested slope is steeper than current one
  bool isConvex = (actualSlopeAngle <= slope);

  // center of the rounding circle
  auto rotCenterPos = isConvex ? d_rootPoint - d_vertNormal * radii : d_rootPoint + d_vertNormal * radii;
  // step of the polyline
  auto slopeStep = (actualSlopeAngle - slope) / nPoints;

  
  // add rounding points
  for (int i = 0; i < nPoints; ++i)
  {
    if (isConvex)
    {
      d_genPoints.push_back(rotCenterPos + d_vertNormal * radii * std::cos(i * slopeStep)
        - d_planeDirection * radii * std::sin(i * slopeStep));
    }
    else
    {
      d_genPoints.push_back(rotCenterPos - d_vertNormal * radii * std::cos(i * slopeStep)
        + d_planeDirection * radii * std::sin(i * slopeStep));
    }
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////
double Generatrix::getLowerZ() const
{
  return static_cast<double>( d_genPoints.back().z );
}

////////////////////////////////////////////////////////////////////////////////
bool Generatrix::buildBottomPart(double radii, double slope, int nPointsVert, int nPointsRound, double shift)
{
  if (shift != 0.0)
    for (auto& point : d_genPoints)
    {
      point.z += shift;
    }

  auto lowZ = getLowerZ();
  double straightHeight = lowZ - radii * (1.0 - std::cos(slope));
  double straightLength = straightHeight / std::sin(slope);
  double straightStep = straightLength / (nPointsVert + 1);

  auto lastPoint = d_genPoints.back();

   
  // horizontal direction in the generatrix plane
  Math_NS::Vector3P inplaneHor( d_planeDirection.x, d_planeDirection.y, 0.);
  inplaneHor = inplaneHor.makeUnit();
  

  double horStep = -straightStep * std::cos(slope);
  double vertStep = straightStep * std::sin(slope);

  for (int i = 1; i <= nPointsVert; ++i)
  {
    d_genPoints.push_back(lastPoint - inplaneHor * i * horStep - Math_NS::Vector3P::zAx() * i * vertStep);
  }

  lastPoint = d_genPoints.back();

  auto rotCenter = lastPoint + inplaneHor * std::sin(slope) * radii + Math_NS::Vector3P::zAx() * std::cos(slope) * radii;
  auto angleStep = slope / nPointsRound;

  double startZ = d_genPoints.back().z;

  for (int i = 0; i < nPointsRound; ++i)
  {
    d_genPoints.push_back(rotCenter - inplaneHor * radii * std::sin(slope - angleStep * i) - Math_NS::Vector3P::zAx() * radii * std::cos(slope - angleStep * i));
  }


  double deltaZ = d_genPoints.back().z - startZ;
  return true;
}
