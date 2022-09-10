// Copyright JGX Software Solutions, 2018.

// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
// THIS SOFTWARE.



//MPRUT: NOTE: the functionality is enhanced to use internal CMesh class. Also the simple function of the ASCII STL export is added 

#include "STLImport.h"
#include "../MathTypes/Vector.h"
#include "../Mesh/Mesh.h"

#include <fstream>
#include <iostream>
#include <string>

using namespace std;

struct vertex {
	float x;
	float y;
	float z;
};

vertex operator ^ (const vertex &, const vertex &);
vertex operator - (const vertex &, const vertex &);
void unit(vertex &);

bool isBinarySTL(char *);

void STLAddFacet(const vertex& v1, const vertex& v2, const vertex& v3, const vertex& normal, CMesh_NS::CMesh& mesh);

void STLImport(const std::filesystem::path & fileName, CMesh_NS::CMesh& mesh ) 
{

	// Import a binary STL file.

	int nVertex = 0; // Number of vertices read.
	int nFacet = 0;  // Number of facets read.

	// Open the file for reading using an input fstream.

	ifstream ifs(fileName, ifstream::binary);

	// Get pointer to the associated buffer object.
	// rdbuf returns a streambuf object associated with the
	// input fstream object ifs.

	filebuf* pbuf = ifs.rdbuf();

	// Calculate the file's size.

	auto size = pbuf->pubseekoff(0, ifs.end);

	// Set the position pointer to the beginning of the file.

	pbuf->pubseekpos(0);

	// Allocate memory to contain file data.

	char* buffer = new char[(size_t)size];

	// Get file data. sgetn grabs all the characters from the streambuf 
	// object 'pbuf'. The return value of sgetn is the number of characters
	// obtained - ordinarily, this value should be checked for equality
	// against the number of characters requested.

	pbuf->sgetn(buffer, size);

	// Test to see if the file is binary.

	if (!isBinarySTL(buffer)) return;

	char * bufptr = buffer;

	bufptr += 80;  // Skip past the header.
	bufptr += 4;   // Skip past the number of triangles.

	vertex normal;
	vertex v1, v2, v3;

	while (bufptr < buffer + size) {

		normal.x = *(float *)(bufptr);
		normal.y = *(float *)(bufptr + 4);
		normal.z = *(float *)(bufptr + 8);
		bufptr += 12;

		v1.x = *(float *)(bufptr);
		v1.y = *(float *)(bufptr + 4);
		v1.z = *(float *)(bufptr + 8);
		bufptr += 12;

		v2.x = *(float *)(bufptr);
		v2.y = *(float *)(bufptr + 4);
		v2.z = *(float *)(bufptr + 8);
		bufptr += 12;

		v3.x = *(float *)(bufptr);
		v3.y = *(float *)(bufptr + 4);
		v3.z = *(float *)(bufptr + 8);
		bufptr += 12;

		const float eps = (float) 1.0e-9;

		// If the normal in the STL file is blank, then create a proper normal.

		if (abs(normal.x) < eps && abs(normal.y) < eps && abs(normal.z) < eps) {
			vertex u, v;
			u = v2 - v1;
			v = v3 - v1;
			normal = u ^ v;
			unit(normal);
		}

		nFacet++;
		nVertex += 3;

		STLAddFacet(v1, v2, v3, normal, mesh);
        
		bufptr += 2;
	}


	ifs.close();

	delete[] buffer;
}


void STLAddFacet(const vertex& v1, const vertex& v2, const vertex& v3, const vertex& normal, CMesh_NS::CMesh& mesh) {


	// STL facet implementation goes here.
  
  mesh.addTrianlge(Math_NS::Vector3P(v1.x, v1.y, v1.z), Math_NS::Vector3P(v2.x, v2.y, v2.z), Math_NS::Vector3P(v3.x, v3.y, v3.z));

}

vertex operator ^ (const vertex & a, const vertex & b) {
	// Cross product.
	vertex result;
	result.x = a.y * b.z - a.z * b.y;
	result.y = a.z * b.x - a.x * b.z;
	result.z = a.x * b.y - a.y * b.x;
	return(result);
}

vertex operator - (const vertex & a, const vertex & b) {
	// Subtraction.
	vertex result;
	result.x = a.x - b.x;
	result.y = a.y - b.y;
	result.z = a.z - b.z;
	return(result);
}

void unit(vertex & v) {
	// Normalize a vector.
	float vmod = pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2);
	vmod = sqrt(vmod);

	if (vmod > (float)1.0e-9) {
		v.x /= vmod;
		v.y /= vmod;
		v.z /= vmod;
	}
}



bool isBinarySTL(char * buffer) {

  // Determine if a file is binary STL.

  bool bbinary = true;
  size_t spnsz, spnsz0;

  // Look for the first non-space character.

  spnsz = strspn(buffer, " ");

  char ctstr[6];  // Enough space to hold "solid\0" and "facet\0".

  // Copy the first five characters from the location of the first non-space
  // character to ctstr.

  strncpy_s(ctstr, &buffer[spnsz], 5);

  ctstr[5] = '\0';
  char csolid[] = "solid\0";

  // If this is an ASCII STL file, then the first string should be "solid".

  if (!strcmp(ctstr, csolid)) {
    // This file might be binary or text. To be certain we need to do a further test.
    // Read past the next new line. If this is a text file, there should be a newline.
    // The next token should be 'facet'.

    spnsz0 = 5 + spnsz;

    char * pch = strchr(&buffer[spnsz0], '\n');  // Look for the first instance of '\n'.
    // If pch is NULL then this is a binary STL file.
    if (pch) {
      pch++;

      spnsz = strspn(pch, " "); // Look for the first instance not of ' '.
      spnsz0 = spnsz;

      spnsz = strcspn(pch + spnsz0, " "); // Look for the first instance of ' '.

      if (spnsz == 5) {
        // Check for 'facet'.
        strncpy_s(ctstr, pch + spnsz0, 5);
        ctstr[5] = '\0';

        char cfacet[] = "facet\0";
        if (!strcmp(ctstr, cfacet)) {
          // This file is beyond reasonable doubt ASCII STL.
          bbinary = false;
        }
      }
    }
  }

  return(bbinary);
}



void exportFace( const CMesh_NS::Face& face, const CMesh_NS::CMesh& i_mesh, std::ofstream& file )
{
  auto points = i_mesh.getPoints(face);
  Math_NS::Vector3P normal = ((points.p2 - points.p1) ^ (points.p3 - points.p2)).makeUnit();

  auto n = normal.toArray<float>();
  auto p1 = points.p1.toArray<float>();
  auto p2 = points.p2.toArray<float>();
  auto p3 = points.p3.toArray<float>();

  file << "\t" << "facet normal" << " " << n[0] << ' ' << n[1] << ' ' << n[2] << std::endl;
  file << "\t\t" << "outer loop" << std::endl;
  file << "\t\t\t" << " " << "vertex" << " " << p1[0] << ' ' << p1[1] << ' ' << p1[2] << std::endl;
  file << "\t\t\t" << " " << "vertex" << " " << p2[0] << ' ' << p2[1] << ' ' << p2[2] << std::endl;
  file << "\t\t\t" << " " << "vertex" << " " << p3[0] << ' ' << p3[1] << ' ' << p3[2] << std::endl;
  file << "\t\t" << "endloop" << std::endl;
  file << "\t" << "endfacet" << std::endl;
}

  
  void STLExport( const CMesh_NS::CMesh& i_mesh, const std::filesystem::path& fileName, const std::string& name /*= ""*/)
  {
    std::ofstream file(fileName.generic_string());
    file << "solid" << " " << name << std::endl;
    for (const auto& face : i_mesh.getFaceList()) 
    {
      exportFace(face, i_mesh, file);
    }
    file << "endsolid" << " " << name << std::endl;

    file.close();
  }

  void STLExport(const CMesh_NS::CMesh& i_mesh, 
                  const std::set<CMesh_NS::FaceID>& faceSet, 
                  const std::filesystem::path& fileName, 
                  const std::string& name /*= ""*/)
  {
    std::ofstream file(fileName.generic_string());
    file << "solid" << " " << name << std::endl;
    for (const auto& faceID : faceSet)
    {
      exportFace(i_mesh.getFace(faceID), i_mesh, file);
    }
    file << "endsolid" << " " << name << std::endl;

    file.close();
  }


