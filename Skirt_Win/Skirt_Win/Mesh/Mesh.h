// Copyright Maxim Prut klocska@gmail.com, 2022.

// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
// THIS SOFTWARE.

#pragma once

#include "../MathTypes/TransMat.h"
#include "../MathTypes/Vector.h"

#include<array>
#include<optional>
#include<set>
#include<tuple>
#include<vector>


/// Contains a class hierarchy to define a connected triangulated mesh
/// The implementation is not the optimal from the memory usage point of view (in comparison with QEdge)
/// but taking to account the potential part dimension it allows me to have a data structure is adapted to task and avoid
/// implementation (or import) of AABB trees or balanced 3D trees. That, definitely, rather expensive
/// The mesh object has a bunch of topological and connectivity information and automatically knows its boundary as a set of orphan coedges
/// It also partially solves a problem of degenerated triangles and (somehow) inverted normals if triangles are added consequently
/// that helps me to process stl files.
/// Also it has the export of data for rendering that can be used for OGL vertex lists approach
namespace CMesh_NS
{
  static constexpr int Orphan = -1;

  using VertID = int;
  using EdgeID = int;
  using FaceID = int;

  using PointID = int;

  static bool isOrphan( int id)
  {
    return id == Orphan;
  };

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// Topological Vertex. Can belong by to coEdges
  class Vertex
  {
  public:
    Vertex() = delete;
    Vertex(PointID myPoint) : d_point(myPoint) { };

    PointID getPointID() const noexcept
    {
      return d_point;
    }

    void addEdgeRef(EdgeID id) noexcept
    {
      myEdges.push_back(id);
    };

    const std::vector<EdgeID>& getEdges() const noexcept
    {
      return myEdges;
    }

  private:
    PointID d_point;
    std::vector<EdgeID> myEdges;
  };

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// Topological coEdge. Can belong by one Face. Can refer to conjugated coEdge and next coEdge for the Face
  class CoEdge
  {
  public:
    CoEdge() = delete;
    CoEdge(VertID v1, VertID v2) : d_v1(v1), d_v2(v2) {};

    VertID getV1() const noexcept {
      return d_v1;
    }

    VertID getV2() const noexcept {
      return d_v2;
    }

    void setNext(EdgeID next) noexcept
    {
      d_next = next;
    }
    
    EdgeID getNext() const noexcept
    {
      return d_next;
    }

    void setCoEdge(EdgeID coEdge) noexcept
    {
      d_coE = coEdge;
    }

    EdgeID getCoEdge() const noexcept
    {
      return d_coE;
    }

    void setFace(FaceID face) noexcept
    {
      d_face = face;

    }

    FaceID getFace() const noexcept
    {
      return d_face;
    }

  private:
    VertID d_v1 ;
    VertID d_v2;
    EdgeID d_next = Orphan;
    EdgeID d_coE = Orphan;
    FaceID d_face = Orphan;
  };

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// Topological Face
  class Face
  {
  public:
    std::tuple<EdgeID, EdgeID, EdgeID> getCoedges() const
    {
      return std::make_tuple( coEdges[0], coEdges[1], coEdges[2] );
    }

    Face(EdgeID e1, EdgeID e2, EdgeID e3)
    {
      coEdges = { e1, e2, e3 };
    }

  private:
    std::array<EdgeID, 3> coEdges;
  };

  struct FacePoints
  {
    const Math_NS::Vector3P& p1;
    const Math_NS::Vector3P& p2;
    const Math_NS::Vector3P& p3;
  };
    

  //////////////////////////////////////////////////////////////////////////////////////////////
  class CMesh
  {
  public:
    /// Add a triangle to the mesh. The triangle will be connected to the adjacent faces if points are closer than tolerance
    /// If triangle contains degenerated edges it will be skipped. Current implementation extremely likes the cases when
    /// the next triangle has contact with set of previous ones (but will work an any case)
    void addTrianlge(const Math_NS::Vector3P& p1, const Math_NS::Vector3P& p2, const Math_NS::Vector3P& p3, double mergeTol = 1.0E-6);

    /// return vertex by ID
    const Vertex& getVertex(VertID id) const
    {
      return d_vertices.at(id);
    }

    /// return vertex by ID
    Vertex& getVertex(VertID id)
    {
      return d_vertices.at(id);
    }

    /// return coEdge by ID
    const CoEdge& getCoEdge(EdgeID id) const
    {
      return d_edges.at(id);
    }

    /// return coEdge by ID
    CoEdge& getCoEdge(EdgeID id)
    {
      return d_edges.at(id);
    }

    /// return Face by ID
    const Face& getFace(FaceID id) const
    {
      return d_faces[id];
    }

    /// return Face by ID
    Face& getFace(FaceID id)
    {
      return d_faces[id];
    }

    /// return geometrical point by ID
    const Math_NS::Vector3P& getPoint(PointID id) const
    {
      return d_points.at(id);
    }

    /// return geometrical point by ID
    Math_NS::Vector3P& getPoint(PointID id)
    {
      return d_points.at(id);
    }

    const std::vector<Math_NS::Vector3P>& getAllPoints() const noexcept
    {
      return d_points;
    };

    std::pair<const Math_NS::Vector3P&, const Math_NS::Vector3P&> getEdgePoints(EdgeID id) const
    {
      return { getPoint(getCoEdge(id).getV1()), getPoint(getCoEdge(id).getV2()) };
    }

    /// return a collection of geometrical points for the face
    FacePoints getPoints( const Face& face) const
    {
      const auto& [e1,e2,e3] = face.getCoedges();
      auto& coE1 = getCoEdge(e1);
      auto& coE2 = getCoEdge(e2);

      return FacePoints{ getPoint(getVertex(coE1.getV1()).getPointID()),
        getPoint(getVertex(coE1.getV2()).getPointID()),
        getPoint(getVertex(coE2.getV2()).getPointID())
      };
    }

    /// collects all adjacent faces for the vertex
    std::set<FaceID> getAdjucentFaces(VertID vertId) const;

    /// return adjacent face for edge
    FaceID getCoFace(EdgeID edge) const
    {
      return getCoEdge( getCoEdge(edge).getCoEdge() ).getFace();
    }

    /// return adjacent face for the next edge
    FaceID getCoFaceV2(EdgeID edge) const
    {
      return getCoEdge( getCoEdge( getCoEdge(edge).getNext() ).getCoEdge() ).getFace();
    }

    /// return adjacent face for the previous edge
    FaceID getCoFaceV1(EdgeID edge) const
    {
      EdgeID v1Edge = getCoEdge( getCoEdge(edge).getNext() ).getNext();
      return getCoEdge( getCoEdge(v1Edge).getCoEdge() ).getFace();
    }

    /// evaluates a non-unit normal vector
    /// @param[in] face FaceID
    /// return normal vector
    Math_NS::Vector3P evaluateRawNormal(FaceID face) const
    {
      return evaluateRawNormal( getFace(face) );
    }

    /// evaluates mean normal for the vertex. It's evaluated as a mean normal of all adjacent faces
    /// areas of them are ignored
    Math_NS::Vector3D evaluateMeanNormal(VertID vertex) const;
    

    /// Evaluates a non-unit normal for the face. Should be normalized
    Math_NS::Vector3P evaluateRawNormal( const Face& face) const;

    /// returns a vector of faces
    const std::vector<Face>& getFaceList() const
    {
      return d_faces;
    }


    struct RenderTraits
    {
      std::vector<float> points;  /// points in order x1,y1,z1,x2,y2,z2....
      std::vector<int> triangles; /// triangles in order p11, p12, p13, p21, p22, p23...
    };

    /// collect data for render tools
    RenderTraits exportToRender() const noexcept;

    const std::set<EdgeID>& getOrphanList() const noexcept
    {
      return d_orphanEdges;
    };

    /// returns the next orphan edge for the current orphan (edge is connected through vertex V2) or nullopt
    /// if there are more than 2 orphan edges for the vertex only first in the edge list will be returned
    std::optional<EdgeID> findNextOrphan(EdgeID curId) const;

    /// returns unit direction of the edge by ID
    Math_NS::Vector3P findEdgeDir(EdgeID curId) const
    {
      auto[p1, p2] = getEdgePoints(curId);
      return (p2 - p1).makeUnit();
    }

    CMesh applyTransform(const Math_NS::TransMat4P& transMat) const;

  private:
    // adds point and vertex for it, returns appropriate ID.
    // The check if the vertex is unique is on user!
    VertID addVertex(const  Math_NS::Vector3P& point);

    void bindCoEdges(EdgeID coE1, EdgeID CoE2);

    EdgeID addEdge(VertID v1, VertID v2);

    FaceID addFace(EdgeID e1, EdgeID e2, EdgeID e3);

    void cleanUpOrphan( const std::vector<EdgeID>& ids);

    std::vector<Math_NS::Vector3P> d_points;
    std::vector<Vertex> d_vertices;
    std::vector<CoEdge> d_edges;
    std::vector<Face> d_faces;

    std::set<EdgeID> d_orphanEdges; // can be replaced by unordered set. It will speed up functionality but introduces the platform dependent behavior
  };

} //namespace
