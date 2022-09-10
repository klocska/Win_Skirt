// Copyright Maxim Prut klocska@gmail.com, 2022.

// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
// THIS SOFTWARE.

#pragma once

#include "Vector.h"

#include <vector>

//#include <../boost_1_80_0/boost/multiprecision/gmp.hpp>


namespace Math_NS
{
  /// transformation matrix. At the moment only transformation of vector is available. 
  template <class T> 
  class TransMat4
  {
  public:

    /// Unique constructor
    TransMat4()
    {
      coefs.resize(16, 0);
      coefs[pos(0, 0)] = 1.0;
      coefs[pos(1, 1)] = 1.0;
      coefs[pos(2, 2)] = 1.0;
      coefs[pos(3, 3)] = 1.0;
    }


    /// Constructor with basis
    /// @param[in] basis1 x axis for new basis
    /// @param[in] basis2 y axis for new basis
    /// @param[in] basis3 z axis for new basis
    TransMat4(const Vector3<T>& basis1, const Vector3<T>& basis2, const Vector3<T>& basis3)
    {
      coefs.resize(16, 0);

      setVal(0, 0, basis1.x);
      setVal(0, 1, basis1.y);
      setVal(0, 2, basis1.z);
      setVal(0, 3, 0);

      setVal(1, 0, basis2.x);
      setVal(1, 1, basis2.y);
      setVal(1, 2, basis2.z);
      setVal(1, 3, 0);

      setVal(2, 0, basis3.x);
      setVal(2, 1, basis3.y);
      setVal(2, 2, basis3.z);
      setVal(2, 3, 0);

      setVal(3, 0, 0);
      setVal(3, 1, 0);
      setVal(3, 2, 0);
      setVal(3, 3, 1);
    }

    /// Constructor with shift
    /// @param[in] shift - shift vector
    TransMat4(const Vector3<T>& shift)
    {
      coefs.resize(16, 0);

      setVal(0, 0, 1);
      setVal(0, 1, 0);
      setVal(0, 2, 0);
      setVal(0, 3, shift.x);

      setVal(1, 0, 0);
      setVal(1, 1, 1);
      setVal(1, 2, 0);
      setVal(1, 3, shift.y);

      setVal(2, 0, 0);
      setVal(2, 1, 0);
      setVal(2, 2, 1);
      setVal(2, 3, shift.z);

      setVal(3, 0, 0);
      setVal(3, 1, 0);
      setVal(3, 2, 0);
      setVal(3, 3, 1);
    }

    /// Transformation for a vector
    /// @param[in] vector - vector to transform
    /// return transformed vector
    Vector3<T> operator * (const Vector3<T>& vector) const noexcept
    {
      std::vector<T> tempVector{vector.x, vector.y, vector.z, 1};
      std::vector<T> tempResult{0, 0, 0, 0};

      for (int i = 0; i < 4; ++i)
      {
        for (int j = 0; j < 4; ++j)
        {
          tempResult[i] += coefs[pos(i,j)] * tempVector[j];
        }
      }

      Vector3<T> output;
      output.x = tempResult[0];
      output.y = tempResult[1];
      output.z = tempResult[2];

      return output;
    }

  private:
    // evaluate position for i,j indexes
    inline size_t pos(int i, int j) const noexcept
    {
      return i * 4 + j;
    }

    void setVal(int i, int j, double val)
    {
      coefs[pos(i, j)] = val;
    };

    std::vector<T> coefs;
  };

  using TransMat4D = TransMat4<double>;
  using TransMat4P = TransMat4<double/*boost::multiprecision::mpf_float*/>;
}
