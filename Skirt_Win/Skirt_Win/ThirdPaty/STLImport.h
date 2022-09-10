// Copyright Maxim Prut klocska@gmail.com, 2022.

// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
// THIS SOFTWARE.

#pragma once

#include <filesystem>
#include <set>
#include <string>

namespace CMesh_NS
{
  class CMesh;
};

/// these functions are mainly copypasted from different internet sources. I preserved the main header of that opensource code in c++

/// Import of ASCII or binary files
/// @param[in] fileName - seems obvious
/// @param[out] Mesh - CMesh object to add triangles (if it's not empty the previous content will be somehow joined with new one)
void STLImport(const std::filesystem::path & fileName, CMesh_NS::CMesh& mesh);

/// Export of ASCII files
/// @param[in] Mesh - CMesh object to add triangles. The internal type -> double conversion will be performed.
/// @param[in] fileName - seems obvious
/// @param[in] name - name of solid to be saved in the file
void STLExport(const CMesh_NS::CMesh& i_mesh, const std::filesystem::path& fileName, const std::string& name = "");

void STLExport(const CMesh_NS::CMesh& i_mesh,
  const std::set<int>& faceSet,
  const std::filesystem::path& fileName,
  const std::string& name = "");
