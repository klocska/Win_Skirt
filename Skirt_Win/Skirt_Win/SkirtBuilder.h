// Copyright Maxim Prut klocska@gmail.com, 2022.

// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
// THIS SOFTWARE.

#pragma once

#include "Mesh/Mesh.h"

#include <optional>

namespace CMesh_NS
{
  class CMesh;
};

static constexpr double PI = 3.1415926;

/// class to define one generatrix for the skirt mesh
class Generatrix
{
public:
  bool buildTopPart(double radii, double slope, int nPoints);

  double getLowerZ() const;

  bool buildBottomPart(double radii, double slope, int nPointsVert, int nPointsRound, double shift);

  const std::vector<Math_NS::Vector3D>& getPoints() const noexcept
  {
    return d_genPoints;
  }

  Generatrix(const Math_NS::Vector3P& rootPoint, 
    const Math_NS::Vector3P& vertNormal, 
    const Math_NS::Vector3P& planeDirection) : d_rootPoint(rootPoint),
                                              d_vertNormal(vertNormal), 
                                              d_planeDirection(planeDirection)
  {
  }

private:
  
  Math_NS::Vector3P d_rootPoint;
  Math_NS::Vector3P d_vertNormal;
  Math_NS::Vector3P d_planeDirection;

  std::vector<Math_NS::Vector3P> d_genPoints;
};

using Pack = std::vector<Generatrix>;


/// builds skirt for the oriented mesh.
/// Prototype assumptions: 
///   1) Mesh is oriented on a such a manner that the boundary doesn't contain triangles with horizontal normal 
///      (I don't want to cover degeneracies here even though there is some naive functionality for it)
///   2) Mesh is topologically correct. I.e there is no oriented triangles and degeneracies

class SkirtBuilder
{
public:
  struct Data
  {
    double upperSmoothR = 5.;   // upper smoothing radii
    double lowerSmoothR = 5.;   // lower smoothing radii
    double minimalHeight = 0.1;  // minimal distance from any point of body to the bottom

    double slopeAngle = PI / 3; //slope angle (from horizontal plane in radians)

    int straightSegments = 5; // number of segments for straight parts (since it's fixed straight part shouldn't be too small)
    int angularSegments = 15; // number of segments forr upper and bound roundings. They also shouldn't be too small
  };

  enum class ErrorCode
  {
    OK,
    BAD_MESH,
    FAIL
  };

  struct Report
  {
    ErrorCode status = ErrorCode::OK;
    std::optional<CMesh_NS::CMesh> result; 
  };

  /// Run the builder
  Report run(const Data& data);
  
public:
  SkirtBuilder(const CMesh_NS::CMesh& i_mesh) : d_sourceMesh(i_mesh)
  {
  }

private:
  // creates class of generatrix and builds the top smoothing
  void buildGeneratrixTop(const Math_NS::Vector3P& rootPoint, 
    const Math_NS::Vector3P& vertNormal, 
    const Math_NS::Vector3P& planeDir, 
    Pack& pack);

  // builds bottom part of generatrix with smoothing and shifts it up using packShift
  void buildGenBottom(Pack& pack, double packShift);

  struct PlaneParams
  {
    Math_NS::Vector3P startVertNormal; // "vertical" normal to start generatrix (normal to inclined source)
    Math_NS::Vector3P startDirOnPlane; // direction of plane for the start generatrix

    Math_NS::Vector3P intermediateVertNormal; // "vertical" normal for intermediate plane
    Math_NS::Vector3P intermediateDirOnPlane; // direction for the intermediate plane

    Math_NS::Vector3P nextVertNormal; // "vertical" normal for the next edge based generatrix()
    Math_NS::Vector3P nextDirOnPlane; // direction for the next edge based generatrix
     
    Math_NS::Vector3P startPoint; // start point. All generatrix are built form one with different normals and directions
    Math_NS::Vector3P vertexNormal; // normal to vertex that is used as a start point
  };

  // evaluates parameters of the plane is used for the particular generatrix.
  PlaneParams evaluatePlane(CMesh_NS::EdgeID edgeID, CMesh_NS::EdgeID nextEdgeID);

  // generates triangles for the pack of generatrixes and adds them to main mesh
  CMesh_NS::CMesh renderPackToMesh(const std::vector<Pack>& packSet, const Math_NS::TransMat4P& transform) const;

  // finds lowes z coordinate for pack of generatrixes
  static double findLowestInPacks(const std::vector<Pack>& packSet);

  struct NormAndDir
  {
    Math_NS::Vector3P direction;
    Math_NS::Vector3P normal;
  };

  // evaluates normal and direction for the generatrix start for the particular vertex normal, edge chord and (if it's known) plane direction
  static NormAndDir evaluateNormAndDir(const Math_NS::Vector3P& vertNormal,
    const Math_NS::Vector3P& edgeChord, const std::optional<Math_NS::Vector3P>& knownDir);

  // builds serie of generatrixes
  void expandDirections(const PlaneParams& params, Pack& pack);
  
  const CMesh_NS::CMesh& d_sourceMesh;
  double d_minZ = std::numeric_limits<double>::max();
  Data d_curData;
};
