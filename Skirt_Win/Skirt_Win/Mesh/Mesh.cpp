// Copyright Maxim Prut klocska@gmail.com, 2022.

// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
// THIS SOFTWARE.

#include "Mesh.h"

#include "../MathTypes/Vector.h"

#include <numeric>

//#pragma optimize ("",off)

namespace CMesh_NS
{

  enum class HitHint
  {
    NONE,
    V1,
    V2,
    BOTH
  };


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // The idea behind this mess is to have a set of connected triangles with topologically consistent orientation and without degeneracies
  // on the any step the orphenEdges list contains a set of edges without coedge. It allows to reduce search and, later, 
  // when we have a connected component to be able to walk along boundary without additional operations
  void CMesh::addTrianlge(const Math_NS::Vector3P& p1, const Math_NS::Vector3P& p2, const Math_NS::Vector3P& p3, double mergeTol /*= 1.0E-8*/)
  {
    int edgeSize = isize(d_edges);

    const double MergeTolSqr = mergeTol * mergeTol;

    // degeneracy. Ignore
    if (distSqr(p1, p2) < mergeTol || distSqr(p2, p3) < mergeTol || distSqr(p3, p1) < mergeTol)
      return;

    EdgeID coE1 = Orphan;
    EdgeID coE2 = Orphan;
    EdgeID coE3 = Orphan;
    
    VertID fV1 = Orphan;
    VertID fV2 = Orphan;
    VertID fV3 = Orphan;
               
    auto isHit = [this, mergeTol](EdgeID edge, const Math_NS::Vector3P& point)
    {
      HitHint hitHint = HitHint::NONE;
      const auto& [eP1, eP2] = getEdgePoints(edge);

      if (Math_NS::distSqr(eP1, point) < mergeTol)
        hitHint = HitHint::V1;

      if (Math_NS::distSqr(eP2, point) < mergeTol)
        hitHint = hitHint == HitHint::V1 ? HitHint::BOTH : HitHint::V2;

      return hitHint;
    };

    bool sence = true;
             
    // collect hits by edges and points to find coEdges
    for (EdgeID orphanEdgeID : d_orphanEdges)
    {
      const auto& orphanEdge = getCoEdge(orphanEdgeID);

      VertID v1 = Orphan;
      VertID v2 = Orphan;
      VertID v3 = Orphan;
      
      HitHint hitResult = isHit(orphanEdgeID, p1);
                 
      if (hitResult == HitHint::V1)
        v1 = orphanEdge.getV1();
      if (hitResult == HitHint::V2)
        v1 = orphanEdge.getV2();

      hitResult = isHit(orphanEdgeID, p2);

      if (hitResult == HitHint::V1)
        v2 = orphanEdge.getV1();
      if (hitResult == HitHint::V2)
        v2 = orphanEdge.getV2();

      hitResult = isHit(orphanEdgeID, p3);

      if (hitResult == HitHint::V1)
        v3 = orphanEdge.getV1();
      if (hitResult == HitHint::V2)
        v3 = orphanEdge.getV2();

      if (!isOrphan(v1) && !isOrphan(v2))
      {
        coE1 = orphanEdgeID;
        sence = (v1 != orphanEdge.getV1());
      }
      else if (!isOrphan(v2) && !isOrphan(v3))
      {
        coE2 = orphanEdgeID;
        sence = (v2 != orphanEdge.getV1());
      }
      else if (!isOrphan(v3) && !isOrphan(v1))
      {
        coE3 = orphanEdgeID;
        sence = (v3 != orphanEdge.getV1());
      }

      if (!isOrphan(v1))
        fV1 = v1;

      if (!isOrphan(v2))
        fV2 = v2;

      if (!isOrphan(v3))
        fV3 = v3;
      
    }

    // add all vertices that were not found
    if( isOrphan(fV1) )
      fV1 = addVertex( p1 );
    if( isOrphan(fV2) )
      fV2 = addVertex( p2 );
    if( isOrphan(fV3) )
      fV3 = addVertex( p3 );

    // add edges and connectivity
    EdgeID e3 = sence ? addEdge(fV3, fV1) : addEdge(fV1, fV3);
    bindCoEdges(e3, coE3);
        
    EdgeID e2 = sence ? addEdge(fV2, fV3) : addEdge(fV3, fV2);
    bindCoEdges(e2, coE2);
    getCoEdge(e2).setNext(e3);

    EdgeID e1 = sence ? addEdge(fV1, fV2) : addEdge(fV2, fV1);
    bindCoEdges(e1, coE1);
    getCoEdge(e1).setNext(e2);

    getCoEdge(e3).setNext(e1);

    FaceID face = addFace(e1, e2, e3);
    
    // clean up orphan list
    if (!isOrphan(coE1) || !isOrphan(coE2) || !isOrphan(coE3))
      cleanUpOrphan( std::vector<EdgeID>{coE1, coE2, coE3} );

    // and add new items if any
    if (isOrphan(coE1))
      d_orphanEdges.emplace(e1);

    if (isOrphan(coE2))
      d_orphanEdges.emplace(e2);

    if (isOrphan(coE3))
      d_orphanEdges.emplace(e3);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  std::set<FaceID> CMesh::getAdjucentFaces(VertID vertId) const
  {
    std::set<FaceID> result;
    const auto& edgeList = getVertex(vertId).getEdges();

    for (EdgeID edgeId : edgeList)
    {
      result.emplace(getCoEdge(edgeId).getFace());
    }

    return result;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Math_NS::Vector3D CMesh::evaluateMeanNormal(VertID vertex) const
  {
    const auto faces = getAdjucentFaces(vertex);
    Math_NS::Vector3D meanNormal;

    std::for_each(faces.begin(), faces.end(), [this, &meanNormal](FaceID id)
    {
      meanNormal = meanNormal + evaluateRawNormal(id).makeUnit();
    });

    meanNormal = meanNormal / isize(faces);
    return meanNormal.makeUnit();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Math_NS::Vector3P CMesh::evaluateRawNormal(const Face& face) const
  {
    const auto& [e1, e2, e3] = face.getCoedges();
        
    VertID v1 = getCoEdge(e1).getV1();
    VertID v2 = getCoEdge(e1).getV2();
    VertID v3 = getCoEdge( getCoEdge(e1).getNext() ).getV2();

    Math_NS::Vector3P p1 = getPoint(v1);
    Math_NS::Vector3P p2 = getPoint(v2);
    Math_NS::Vector3P p3 = getPoint(v3);
        
    return (p2 - p1) ^ (p3 - p1);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  CMesh::RenderTraits CMesh::exportToRender() const noexcept
  {
    RenderTraits result;

    result.points.reserve(d_points.size() * 3);
    result.triangles.reserve(d_faces.size() * 3);

    for (const auto& point : d_points)
    {
      result.points.push_back(static_cast<float>(point.x));
      result.points.push_back(static_cast<float>(point.y));
      result.points.push_back(static_cast<float>(point.z));
    }

    for (const auto& face : d_faces)
    {
      auto[e1, e2, e3] = face.getCoedges();

      result.triangles.push_back(getCoEdge(e1).getV1());
      result.triangles.push_back(getCoEdge(e2).getV1());
      result.triangles.push_back(getCoEdge(e3).getV1());
    }
    return result;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  CMesh CMesh::applyTransform(const Math_NS::TransMat4P& transMat) const
  {
    CMesh newMesh( *this );
    newMesh.d_points.clear();

    for (const auto& point : d_points)
      newMesh.d_points.push_back( transMat * point);

    return newMesh;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  std::optional<EdgeID> CMesh::findNextOrphan(EdgeID curId) const
  {
    const auto& vertex =getVertex(getCoEdge(curId).getV2());
    const auto& adjEdges = vertex.getEdges();

    for (EdgeID edge : adjEdges)
    {
      if (edge == curId)
        continue;

      if (d_orphanEdges.find(edge) != d_orphanEdges.end())
        return edge;
    }

    return std::optional<CMesh_NS::EdgeID>();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  VertID CMesh::addVertex(const Math_NS::Vector3P& point)
  {
    d_points.push_back( point );
    d_vertices.emplace_back( isize(d_points) - 1 );
    return isize(d_vertices) - 1;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void CMesh::bindCoEdges(EdgeID coE1, EdgeID coE2)
  {
    if (coE1 == Orphan || coE2 == Orphan)
      return;

    if( !isOrphan(coE1) )
      getCoEdge(coE1).setCoEdge(coE2);

    if( !isOrphan(coE2) )
      getCoEdge(coE2).setCoEdge(coE1);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  EdgeID CMesh::addEdge(VertID v1, VertID v2)
  {
    d_edges.emplace_back(v1, v2);

    EdgeID myId = isize(d_edges) - 1;

    getVertex(v1).addEdgeRef(myId);
    getVertex(v2).addEdgeRef(myId);
    
    return myId;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  FaceID CMesh::addFace(EdgeID e1, EdgeID e2, EdgeID e3)
  {
    if (isOrphan(e1) || isOrphan(e2) || isOrphan(e3))
      return Orphan;

    d_faces.emplace_back(e1, e2, e3);
    FaceID myId = isize(d_faces) - 1;

    getCoEdge(e1).setFace(myId);
    getCoEdge(e2).setFace(myId);
    getCoEdge(e3).setFace(myId);
    return myId;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void CMesh::cleanUpOrphan(const std::vector<EdgeID>& ids)
  {
    for (EdgeID eID : ids)
    {
      if (!isOrphan(eID))
        d_orphanEdges.erase(eID);
    }
  }
}
