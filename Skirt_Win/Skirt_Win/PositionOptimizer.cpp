// Copyright Maxim Prut klocska@gmail.com, 2022.

// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
// THIS SOFTWARE.


#include "PositionOptimizer.h"

#include "MathTypes/TransMat.h"
#include "MathTypes/Vector.h"
#include "Mesh/Mesh.h"
#include "ThirdPaty/STLIMport.h"

//#pragma optimize("",off)

RotationOptimizer::RotationOptimizer(const CMesh_NS::CMesh& meshFor) : d_meshFor(meshFor)
{

}

RotationOptimizer::Report RotationOptimizer::run(const RotationOptimizer::Data& data)
{
  auto FaceList = d_meshFor.getFaceList();

  Math_NS::Vector3P meanNormal;

  for (const auto& face : FaceList)
  {
    meanNormal = meanNormal + d_meshFor.evaluateRawNormal( face );
  }

  meanNormal = meanNormal.lengh() > 1.0e-12 ? meanNormal.makeUnit() : meanNormal = Math_NS::Vector3P::zAx();

  if (meanNormal.isZero())
    meanNormal.z = 1.0;

  using anglePair = std::pair<int, Pdouble>;
  std::array<anglePair,3> angles;

  angles[0] = std::make_pair<int, Pdouble>(0, meanNormal * Math_NS::Vector3P::xAx());
  angles[1] = std::make_pair<int, Pdouble>(1, meanNormal * Math_NS::Vector3P::yAx());
  angles[2] = std::make_pair<int, Pdouble>(2, meanNormal * Math_NS::Vector3P::zAx());

  std::sort(angles.begin(), angles.end(), [](const anglePair& p1, const anglePair& p2) {return std::abs(p1.second) < std::abs(p2.second); });

  Math_NS::Vector3P newZ = meanNormal;
  Math_NS::Vector3P newY;
  Math_NS::Vector3P newX;

  if (angles.front().first == 0)
    newY = (newZ ^ Math_NS::Vector3P::xAx()).makeUnit();
  else if (angles.front().first == 1)
    newY = (newZ ^ Math_NS::Vector3P::yAx()).makeUnit();
  else
    newY = (newZ ^ Math_NS::Vector3P::zAx()).makeUnit();

  newX = newY ^ newZ;

  Math_NS::TransMat4P candidate(newX, newY, newZ);
    
  Report result;
  result.transform = std::move(candidate);
  
  return result;
}


RotationOptimizer::ErrorCode RotationOptimizer::checkValidity(const Math_NS::TransMat4P& transform, const RotationOptimizer::Data& data)
{
  CMesh_NS::CMesh tempMesh = d_meshFor.applyTransform(transform);

  const auto& orphanList = tempMesh.getOrphanList();
  std::set<CMesh_NS::EdgeID> affectedSet;

  CMesh_NS::EdgeID startEdge = *orphanList.begin();
  CMesh_NS::EdgeID curEdge = startEdge;

  auto dumpError = [&data, &tempMesh](CMesh_NS::FaceID problemFace, const std::string& problemName)
  {
    std::set<CMesh_NS::FaceID> orphanFaces;
    std::filesystem::path dumpPass = data.dumpPath;
    dumpPass.replace_filename(problemName + ".stl");

    orphanFaces.emplace(problemFace);
    STLExport(tempMesh,
      orphanFaces,
      dumpPass);

    std::filesystem::path meshPass = data.dumpPath;
    meshPass.replace_filename("problematic_mesh.stl");
    STLExport(tempMesh, meshPass);
  };

  do
  {
    do
    {
      affectedSet.emplace(curEdge);

      auto orphanFace = tempMesh.getCoEdge(curEdge).getFace();
      auto nextOrphanOpt = tempMesh.findNextOrphan(curEdge);
      
      if (!nextOrphanOpt)
      {
        dumpError(orphanFace, "topology_error");
        return ErrorCode::BadTopology;
      }

      if (tempMesh.evaluateRawNormal(orphanFace) * Math_NS::Vector3P::zAx() <= 0.)
      {
        dumpError(orphanFace, "normal_error");
        return ErrorCode::OverhangingTriangles;
      }

      CMesh_NS::EdgeID nextOrphan = *nextOrphanOpt;
      curEdge = nextOrphan;
    } while (affectedSet.find(curEdge) == affectedSet.end());
  } while (isize(affectedSet) != isize(orphanList));

  return ErrorCode::Ok;
}
