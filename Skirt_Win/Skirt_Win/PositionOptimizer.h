// Copyright Maxim Prut klocska@gmail.com, 2022.

// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
// THIS SOFTWARE.

#pragma once

#include "MathTypes/TransMat.h"

#include <filesystem>

namespace CMesh_NS
{
  class CMesh;
};

/// Rather stupid optimizer that could be enhanced to the optimal material consumption search if it were production code.
/// The only available strategy is a mean normal is oriented along the Z axis. The mean normal is evaluated in accordance with polygons area.
/// Actually I know almost nothing regarding the technological process here so I used a reasonable criteria for the traditional stamps:
/// the main area of the mold should have orientation that is close to z. It also helps to minimize the material usage for the detail.
///
/// UNSOLVED PROBLEM: degeneracies. Now the sample part generates a fold. It can be removed either by the proper orientation (for the current version
/// it may be manual orientation with -opt parameter is set as OFF) or by additional development. Triangles of skirt may be divided by intersections 
/// with other skirt triangles. Unnecessary polygons (tha are lying under top level) can be removed so the top level will form the correct surface.
/// Such a boolean-like operation is quite expensive from the development time point of view so I'd like to omit it since I like to have free time
/// after my work hours ;)

class RotationOptimizer
{
public:
  RotationOptimizer(const CMesh_NS::CMesh& meshFor);

  enum class Strategy
  {
    MeanNormal
  };

  struct Data
  {
    Strategy strategy = Strategy::MeanNormal; // evaluates a mean normal and orients vertically
    std::filesystem::path dumpPath;
  };


  enum class ErrorCode
  {
    Ok,
    OverhangingTriangles,   // the shape has vertical triangles on the boundary. Now it's considered as a show stopper
    BadTopology             // Mesh has bad topology: boundary cannot be deducted
  };

  struct Report
  {
    ErrorCode status = ErrorCode::Ok;
    Math_NS::TransMat4P transform;
  };

  Report run(const Data& data);

  ErrorCode checkValidity(const Math_NS::TransMat4P& transform, const RotationOptimizer::Data& data);
    
private:
  const CMesh_NS::CMesh& d_meshFor;

};
