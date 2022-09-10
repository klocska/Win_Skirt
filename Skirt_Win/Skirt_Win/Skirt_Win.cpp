// Copyright Maxim Prut klocska@gmail.com, 2022.

// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
// THIS SOFTWARE.


#include "PositionOptimizer.h"
#include "Skirt.h"
#include "SkirtBuilder.h"

#include "MathTypes/TransMat.h"
#include "Mesh/Mesh.h"
#include "Thirdpaty/STLImport.h"

#include <filesystem>
#include <iostream>

using namespace std;

// #pragma optimize("",off)

namespace
{
  void printHelp()
  {
    cout << "Usage: Skirt_Win.exe filename.stl [-opt: OFF]" << std::endl;
    cout << "\t [-h help]" << std::endl;
  }

  struct RunData
  {
    std::filesystem::path path;
    bool enableOptimization = true;
  };
  
  // first argument is filename second one may be -opt with third OFF if 
  std::optional<RunData> processArguments(int argc, char* argv[])
  {
    RunData output;

    if (argc < 2)
    {
      printHelp();
      return std::nullopt;
    }

    std::string firstArg(argv[1]);

    if (firstArg == "-h" || firstArg == "-?" || firstArg == "-help")
    {
      printHelp();
      return std::nullopt;
    }

    output.path = argv[1];
    
    if (argc >= 4)
    {
      std::string key2(argv[2]);
      std::string key3(argv[3]);
      if (key2 == "-opt:" && (key3 == "OFF" || key3 == "off"))
      {
        output.enableOptimization = false;
      }
      else if (key2 != "-opt")
      {
        cout << "Incorrect parameter" << std::endl;
        printHelp();
        return std::nullopt;
      }
    }
    return output;
  }
}


int main(int argc, char* argv[])
{
 // technological process parameters. Can be separated to yaml/json/xml file 
 // but I don't want either use third party parser or create creepy functionality. I like to sleep and have a time with family. Sorry ;)
 // For the prototype the units are the same with geometry from the file.
  SkirtBuilder::Data data;
  data.upperSmoothR = 5.; // must be more than 0.1 for the prototype to avoid degeneracies
  data.lowerSmoothR = 5.; // must be more than 0.1 for the prototype
  data.slopeAngle = (1. / 3.) * PI; // supposed not to be overhanging
  data.minimalHeight = 0.1; // must be more than 0.1 to avoid degeneracies

  // get command line arguments parsed
  auto runParams = processArguments(argc, argv);
  if (!runParams)
    return -1;

  auto path = runParams->path;
    
  CMesh_NS::CMesh mesh;
  try
  {
    STLImport(path, mesh);
    cout << "STL import " << runParams->path.generic_string() << " OK" << '\t' << mesh.getFaceList().size() << " faces" << std::endl;
  }
  catch (...)
  {
    cout << "STL import " << path.generic_string() << " FAIL" << '\t' << std::endl;
    cout << "Incorrect filename" << std::endl;
    return -1;
  }
  
  RotationOptimizer::Report optResult;

  RotationOptimizer optimizer(mesh);
  RotationOptimizer::Data optData;
  optData.dumpPath = path;

  // optimize orientation if it's enabled
  if (runParams->enableOptimization)
  {
    optResult = optimizer.run(optData);
    cout << "Orientation optimizer: OK" << endl;
  }
  else
    cout << "Orientation optimizer: OFF" << endl;


  // check is made despite the optimizer call.
  auto optStatus = optimizer.checkValidity(optResult.transform, optData);
  
  if (optStatus == RotationOptimizer::ErrorCode::Ok)
  {
    cout << "Orientation check: OK" << endl;
    
    // get transformed mesh
    auto transfMesh = runParams->enableOptimization ? mesh.applyTransform(optResult.transform) : mesh;
    
    // call skirt builder
    SkirtBuilder builder(transfMesh);
        
    cout << std::endl << "Skirt parameters in the units of the file: " << std::endl;
    cout << '\t' << "Upper smoothing radii: " << data.upperSmoothR << std::endl;
    cout << '\t' << "Lower smoothing radii: " << data.lowerSmoothR << std::endl;
    cout << '\t' << "Slope angle: " << data.slopeAngle << std::endl;
    cout << '\t' << "Minimal height: " << data.minimalHeight << std::endl << std::endl;
    
    auto skirtReport = builder.run(data);
    if (skirtReport.status == SkirtBuilder::ErrorCode::OK)
    {
      cout << "Skirt builder: OK" << endl;

      // output has the same name and location with input file but adds _out to the name
      std::filesystem::path outputFile(path);
      std::string filename = outputFile.filename().generic_string();
      outputFile.replace_filename(filename.substr(0, filename.size() - 4) + "_out.stl") ;
      STLExport(*skirtReport.result, outputFile.generic_string());

      cout << "Results are saved: " << outputFile.generic_string() << '\t' << skirtReport.result->getFaceList().size() << " faces" << endl;
    }
    else
      cout << "Skirt builder: FAIL." <<  endl;

  }
  else
  {
    // diagnostic of the error status
    switch (optResult.status)
    {
    case RotationOptimizer::ErrorCode::BadTopology:
        cout << "Orientation check: FAIL. Bad Topology. See dump" << endl;
        break;
    case RotationOptimizer::ErrorCode::OverhangingTriangles:
      cout << "Orientation check: FAIL. Overhanging triangles. Try manual orientation with optimization is OFF" << endl;
      break;
    }
  }
    

	return 0;
}
