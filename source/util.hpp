#pragma once

#include <algorithm>
#include <Kinect.h>

template<class T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi)
{
	return std::max<T>(lo, std::min<T>(v, hi));
}


struct Vector3
{
	float x, y, z;

	Vector3(float _x, float _y, float _z) :x(_x), y(_y), z(_z) {}

	Vector3(const CameraSpacePoint& p) :x(p.X), y(p.Y), z(p.Z) {}

	Vector3(const Vector3& v)
	{
		this->x = v.x;
		this->y = v.y;
		this->z = v.z;
	}

	Vector3(Vector3&& v)
	{
		this->x = v.x;
		this->y = v.y;
		this->z = v.z;
	}

	Vector3 operator=(const Vector3& v)
	{
		this->x = v.x;
		this->y = v.y;
		this->z = v.z;
		return *this;
	}

	float length() const
	{
		return std::sqrt(x*x + y*y + z*z);
	}

	void normalize()
	{
		const float len = this->length();
		x /= len;
		y /= len;
		z /= len;
	}

	Vector3 operator-(const Vector3& v) const
	{
		return{ x - v.x,y - v.y,z - v.z };
	}

	Vector3 operator+(const Vector3& v) const
	{
		return{ x + v.x,y + v.y,z + v.z };
	}

	Vector3 operator*(const float& scalar) const
	{
		return{ x*scalar,y*scalar,z*scalar };
	}
};



float dot(const Vector3& v1, const Vector3& v2)
{
	return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}



Vector3 cross(const Vector3& v1, const Vector3& v2)
{
	return{ v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x };
}



struct Plane
{
	float a, b, c, d;

	Plane() = default;

	Plane(const Vector3& p1, const Vector3& p2, const Vector3 &p3)
	{
		Vector3 n = cross(p2 - p1, p3 - p1);
		d = dot(n, p1);
		a = n.x;
		b = n.y;
		c = n.z;
	}

	Plane(const Vector3& normal, const Vector3 &p)
	{
		a = normal.x;
		b = normal.y;
		c = normal.z;
		d = dot(normal, p);
	}

	Vector3 normal() const
	{
		return{ a, b, c };
	}
};

float distance(const Plane & plane, const Vector3 & point)
{
	return dot({ plane.a, plane.b, plane.c }, point) + plane.d;
}

float angle(const Plane& p1, const Plane& p2)
{
	auto n1 = p1.normal();
	auto n2 = p2.normal();
	return std::acos(dot(n1, n2) / (n1.length()*n2.length()));
}

template<class Interface>
static inline void SafeRelease(Interface *&interfaceToRelease)
{
	if (interfaceToRelease != nullptr) {
		interfaceToRelease->Release();
		interfaceToRelease = nullptr;
	}
}

//! Interface to manage COM classes in an exception-safe manner
template<class Interface>
class ComWrapper
{
private:
	Interface* iface = nullptr;

public:
	ComWrapper() = default;

	bool isset() const
	{
		return iface == nullptr;
	}

	~ComWrapper()
	{
		SafeRelease(iface);
	}

	Interface** address()
	{
		return &iface;
	}

	Interface*& operator->()
	{
		return iface;
	}
};



template<typename T>
class Bgfx2DMemoryHelper
{
private:
	const bgfx::Memory* mMem;

	int w;
	int h;

public:
	Bgfx2DMemoryHelper(){}
	
	Bgfx2DMemoryHelper(int width, int height, const bgfx::Memory* mem)
		: w(width)
		, h(height)
		, mMem(mem)
	{

	}


	//
	Bgfx2DMemoryHelper(int width, int height)
		: w(width)
		, h(height)
		, mMem(bgfx::alloc(width*height*sizeof(T)))
	{

	}


	//
	T read(int x, int y)
	{
		return reinterpret_cast<T*>(mMem->data)[y*w + x];
	}


	//
	void write(int x, int y, T val)
	{
		T* pointer = reinterpret_cast<T*>(mMem->data);
		pointer[y*w + x] = val;
	}


	Bgfx2DMemoryHelper clone()
	{
		return Bgfx2DMemoryHelper(w,h,bgfx::copy(mMem->data,mMem->size));
	}


	unsigned int width() const
	{
		return w;
	}

	unsigned int height() const
	{
		return h;
	}

	T* raw() const
	{
		return reinterpret_cast<T*>(mMem->data);
	}
};