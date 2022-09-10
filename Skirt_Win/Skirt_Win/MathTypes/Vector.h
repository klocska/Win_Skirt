// Copyright Maxim Prut klocska@gmail.com, 2022.

// THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
// THIS SOFTWARE.

#pragma once

#include <array>

//#include <boost/multiprecision/gmp.hpp>


/// MAth primitive vector. Since I've developed it, I'm going to use it for experiments and add a rational types here in future,

using Pdouble = double;//boost::multiprecision::mpf_float;

template <class T> int isize(const T& container)
{
  return int(container.size());
}

namespace Math_NS
{
  template<typename T> 
  class Vector3
  {
  public:
    T operator * (const Vector3<T>& other) const noexcept
    {
      return x * other.x + y * other.y + z * other.z;
    }

    Vector3 operator * (T value) const noexcept
    {
      return Vector3{ x * value, y * value,  z * value };
    }


    void operator *= (T value) const noexcept
    {
      x *= value;
      y *= value;
      z *= value;
    }

    void operator /= (T value) const
    {
      x /= value;
      y /= value;
      z /= value;
    }

    Vector3 operator / (T value) const
    {
      return Vector3{ x / value, y / value, z / value };
    }

    Vector3<T> operator + (const Vector3<T>& other) const noexcept
    {
      return Vector3{ x + other.x,  y + other.y, z + other.z };
    }

    Vector3<T> operator - (const Vector3<T>& other) const noexcept
    {
      Vector3<T> output;
      output.x = x - other.x;
      output.y = y - other.y;
      output.z = z - other.z;

      return output;
    }
       
    Vector3<T> operator^(const Vector3<T>& other) const noexcept
    {
      Vector3<T> output;
      output.x = y * other.z - z * other.y;
      output.y = z * other.x - x * other.z;
      output.z = x * other.y - y * other.x;

      return output;
    }

    T lenghSqr() const noexcept
    {
      return x * x + y * y + z * z;
    }

    T lengh() const noexcept
    {
      return (T)std::sqrt(x * x + y * y + z * z);
    }

    bool isZero() const noexcept
    {
      return x == 0 && y == 0 && z == 0;
    };

    Vector3<T> makeUnit()
    {
      T magn = std::sqrt(x * x + y * y + z * z);

      if (magn == 0)
        return Vector3<T>();

      Vector3<T> output;
      output.x = x / magn;
      output.y = y / magn;
      output.z = z / magn;

      return output;
    }

    bool isZero()
    {
      return x == 0 && y == 0 && z == 0;
    }

    static Vector3 zero()
    {
      return Vector3{0.,0.,0.};
    }

    static Vector3<T> xAx()
    {
      return Vector3<T>{1, 0, 0};
    }
    static Vector3<T> yAx()
    {
      return Vector3<T>{0, 1, 0};
    }

    static Vector3<T> zAx()
    {
      return Vector3<T>{0, 0, 1};
    }

    
    template <typename OT>
    std::array<OT,3> toArray() const
    {
      std::array<OT, 3> output;

      output[0] = static_cast<OT>(x);
      output[1] = static_cast<OT>(y);
      output[2] = static_cast<OT>(z);

      return output;
    }

    Vector3<T>() = default;

    template<typename Q>
    Vector3<T>(Q i_x, Q i_y, Q i_z)
    {
      x = static_cast<T>(i_x);
      y = static_cast<T>(i_y);
      z = static_cast<T>(i_z);
    }

  public:
    T x = 0;
    T y = 0;
    T z = 0;
  };

  template <typename T> 
  T dist(const Vector3<T>& v1, const Vector3<T>& v2)
  {
    return (v1 - v2).length();
  };

  template <typename T> 
  T distSqr(const Vector3<T>& v1, const Vector3<T>& v2)
  {
    return (v1 - v2).lenghSqr();
  };

  using Vector3D = Vector3 < double >;
  using Vector3P = Vector3 < Pdouble >;
}
