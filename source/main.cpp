#include "common.h"
#include "bgfx_utils.h"
#include "imgui/imgui.h"
#include "camera.h"
#include <opencv2/core.hpp>
#include <bgfx/bgfx.h>
#include "Kinect.h"
#include <stdexcept>
#include <cmath>



struct Vector3
{
	float x, y, z;

	Vector3(float _x, float _y, float _z):x(_x), y(_y), z(_z){}

	Vector3(const CameraSpacePoint& p):x(p.X), y(p.Y), z(p.Z){}

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
	return std::acos(dot(n1,n2)/(n1.length()*n2.length()));
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


//! Synchronous data provider
class KinectDataProvider
{
private:
	ComWrapper<IKinectSensor> sensor;
	ComWrapper<IMultiSourceFrameReader> multiSourceReader;
	ComWrapper<ICoordinateMapper> coordinateMapper;

	const bgfx::Memory* depthBuffer = nullptr;
	int depthBufferWidth = 0;
	int depthBufferHeight = 0;

	const bgfx::Memory* colorBuffer = nullptr;
	int colorBufferWidth = 0;
	int colorBufferHeight = 0;

	IBody* bodies[BODY_COUNT] = { 0 };

public:
	KinectDataProvider()
	{
		// Grab reader
		HRESULT hr = GetDefaultKinectSensor(sensor.address());
		if (sensor.isset() || FAILED(hr))
		{
			throw std::runtime_error("Failed to acquire sensor.");
		}
		hr = sensor->Open();
		if (FAILED(hr))
		{
			throw std::runtime_error("Failed to open sensor.");
		}
		hr = sensor->OpenMultiSourceFrameReader(FrameSourceTypes_Color | FrameSourceTypes_Depth | FrameSourceTypes_Body | FrameSourceTypes_BodyIndex, multiSourceReader.address());
		if (FAILED(hr))
		{
			throw std::runtime_error("Failed to open reader.");
		}
		hr = sensor->get_CoordinateMapper(coordinateMapper.address());
		if (FAILED(hr))
		{
			throw std::runtime_error("Failed to grab coordinate mapper.");
		}

		// Create buffers
		ComWrapper<IMultiSourceFrame> frame;
		while ((hr = multiSourceReader->AcquireLatestFrame(frame.address())) == E_PENDING) {}
		if (FAILED(hr))
		{
			throw std::runtime_error("Failed to grab initial frame.");
		}

		// depth
		{
			ComWrapper<IDepthFrameReference> depthFrameRef;
			hr = frame->get_DepthFrameReference(depthFrameRef.address());
			if (FAILED(hr))
			{
				throw std::runtime_error("Failed to grab initial depth frame reference.");
			}
			ComWrapper<IDepthFrame> depthFrame;
			hr = depthFrameRef->AcquireFrame(depthFrame.address());
			if (FAILED(hr))
			{
				throw std::runtime_error("Failed to grab initial depth frame.");
			}
			ComWrapper<IFrameDescription> depthFrameDesc;
			hr = depthFrame->get_FrameDescription(depthFrameDesc.address());
			if (FAILED(hr))
			{
				throw std::runtime_error("Failed to grab depth frame description.");
			}
			hr = depthFrameDesc->get_Width(&depthBufferWidth);
			if (FAILED(hr))
			{
				throw std::runtime_error("Failed to grab depth frame description.");
			}
			hr = depthFrameDesc->get_Height(&depthBufferHeight);
			if (FAILED(hr))
			{
				throw std::runtime_error("Failed to grab depth frame description.");
			}
			depthBuffer = bgfx::alloc(depthBufferWidth*depthBufferHeight*sizeof(unsigned short));
		}
		
		// color
		{
			ComWrapper<IColorFrameReference> colorFrameRef;
			hr = frame->get_ColorFrameReference(colorFrameRef.address());
			if (FAILED(hr))
			{
				throw std::runtime_error("Failed to grab initial color frame reference.");
			}
			ComWrapper<IColorFrame> colorFrame;
			hr = colorFrameRef->AcquireFrame(colorFrame.address());
			if (FAILED(hr))
			{
				throw std::runtime_error("Failed to grab initial color frame.");
			}
			ComWrapper<IFrameDescription> colorFrameDesc;
			hr = colorFrame->get_FrameDescription(colorFrameDesc.address());
			if (FAILED(hr))
			{
				throw std::runtime_error("Failed to grab color frame description.");
			}
			hr = colorFrameDesc->get_Width(&colorBufferWidth);
			if (FAILED(hr))
			{
				throw std::runtime_error("Failed to grab color frame description.");
			}
			hr = colorFrameDesc->get_Height(&colorBufferHeight);
			if (FAILED(hr))
			{
				throw std::runtime_error("Failed to grab color frame description.");
			}
			colorBuffer = bgfx::alloc(colorBufferWidth*colorBufferHeight*sizeof(RGBQUAD));
		}
	}

	bool RefreshData()
	{
		ComWrapper<IMultiSourceFrame> frame;
		auto hr = multiSourceReader->AcquireLatestFrame(frame.address());
		if (FAILED(hr))
		{
			return false;
		}
		// depth
		{
			ComWrapper<IDepthFrameReference> depthFrameRef;
			hr = frame->get_DepthFrameReference(depthFrameRef.address());
			if (FAILED(hr))
			{
				throw std::runtime_error("Failed to grab depth frame reference.");
			}
			ComWrapper<IDepthFrame> depthFrame;
			hr = depthFrameRef->AcquireFrame(depthFrame.address());
			if (FAILED(hr))
			{
				throw std::runtime_error("Failed to grab depth frame.");
			}
			hr = depthFrame->CopyFrameDataToArray(depthBuffer->size/2, reinterpret_cast<UINT16*>(depthBuffer->data));
			if (FAILED(hr))
			{
				throw std::runtime_error("Failed to copy depth data.");
			}
		}
		// color
		{
			ComWrapper<IColorFrameReference> colorFrameRef;
			hr = frame->get_ColorFrameReference(colorFrameRef.address());
			if (FAILED(hr))
			{
				throw std::runtime_error("Failed to grab color frame reference.");
			}
			ComWrapper<IColorFrame> colorFrame;
			hr = colorFrameRef->AcquireFrame(colorFrame.address());
			if (FAILED(hr))
			{
				throw std::runtime_error("Failed to grab color frame.");
			}
			hr = colorFrame->CopyConvertedFrameDataToArray(colorBuffer->size, reinterpret_cast<BYTE*>(colorBuffer->data), ColorImageFormat_Rgba);
			if (FAILED(hr))
			{
				throw std::runtime_error("Failed to copy color data.");
			}
		}
		

		// body
		{
			ComWrapper<IBodyFrameReference> bodyFrameRef;
			hr = frame->get_BodyFrameReference(bodyFrameRef.address());
			if (FAILED(hr))
			{
				throw std::runtime_error("Failed to grab body frame reference.");
			}
			ComWrapper<IBodyFrame> bodyFrame;
			hr = bodyFrameRef->AcquireFrame(bodyFrame.address());
			if (FAILED(hr))
			{
				throw std::runtime_error("Failed to grab body frame.");
			}
			hr = bodyFrame->GetAndRefreshBodyData(_countof(bodies), bodies);  
			if (FAILED(hr))
			{ 
				throw std::runtime_error("Failed to grab body data."); 
			}
		}

		return true;
	}

	const bgfx::Memory* LatestColorData() const
	{
		return this->colorBuffer;
	}

	const int ColorDataWidth() const
	{
		return this->colorBufferWidth;
	}

	const int ColorDataHeight() const
	{
		return this->colorBufferHeight;
	}

	const bgfx::Memory* LatestDepthData() const
	{
		return this->depthBuffer;
	}

	IBody* LatestBodyData(int bodyIdx)
	{
		return bodies[bodyIdx];
	}

	const int DepthDataWidth() const
	{
		return this->depthBufferWidth;
	}

	const int DepthDataHeight() const
	{
		return this->depthBufferHeight;
	}

	ICoordinateMapper* CoordinateMapper()
	{
		return *(coordinateMapper.address());
	}

	~KinectDataProvider()
	{
		if (sensor.isset())
		{
			sensor->Close();
		}
	}
};


class RenderImage
{
private:
	bgfx::TextureHandle mHandle;
	const bgfx::Memory* mBuffer;

	const unsigned int mWidth;
	const unsigned int mHeight;

public:
	RenderImage(const unsigned int width, const unsigned int height)
		: mWidth(width), mHeight(height)
	{
		mHandle = bgfx::createTexture2D(width, height, false, 1, bgfx::TextureFormat::RGBA8, BGFX_TEXTURE_NONE);
		mBuffer = bgfx::alloc(width * height * 4);
	}

	bgfx::TextureHandle handle() const
	{
		return mHandle;
	}

	bgfx::TextureFormat::Enum format() const
	{
		bgfx::TextureFormat::RGBA8;
	}

	unsigned int width() const
	{
		return mWidth;
	}

	unsigned int height() const
	{
		return mHeight;
	}

	void writePixel(const int x, const int y, const int red, const int green, const int blue, const int alpha)
	{
		auto pos = 4*(y*width() + x);
		mBuffer->data[pos] = red;
		mBuffer->data[pos+1] = green;
		mBuffer->data[pos+2] = blue;
		mBuffer->data[pos+3] = alpha;
	}

	void writePixel(const int x, const int y, const int grey)
	{
		writePixel(x, y, grey, grey, grey, 255);
	}

	void writePixel(const int x, const int y, const int red, const int green, const int blue)
	{
		writePixel(x, y, red, green, blue, 255);
	}

	void update()
	{
		bgfx::updateTexture2D(mHandle, 0, 0, 0, 0, width(), height(), bgfx::copy(mBuffer->data, mBuffer->size));
	}

	void clear()
	{
		std::memset(mBuffer->data, 0, mBuffer->size);
	}
};



template<class T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi)
{
	return std::max<T>(lo,std::min<T>(v, hi));
}



void DrawQuad(RenderImage& target, unsigned int size, unsigned int x, unsigned int y)
{
	for (int py = y - size / 2; py < y + size / 2; py++)
	{
		auto targety = clamp<int>(py, 0, target.height() - 1);
		for (int px = x - size / 2; px < x + size / 2; px++)
		{
			auto targetx = clamp<int>(px, 0, target.width() - 1);
			target.writePixel(targetx, targety, 255, 0, 0);
		}
	}
}


void DrawVector(RenderImage& target, unsigned int x, unsigned int y, float vx, float vy)
{
	const auto xend = x + vx;
	const auto yend = y + vy;
	const auto dx = xend - x;
	const auto dy = yend - y;
	auto err = 2 * dy - dx;
	const auto d0 = 2 * dy;
	const auto dN0 = 2 * (dy - dx);

	target.writePixel(x, y, 255);
	while (x < xend)
	{
		x++;
		if (err <= 0)
		{
			err += d0;
		}
		else
		{
			y++;
			err += dN0;
		}
		target.writePixel(x, y, 255);
	}
}


#include <vector>
#include <memory>
#include <numeric>

#include <opencv2/superres.hpp>
#include <eigen/Core>
class ReconstructionState
{
private:
	std::vector<const bgfx::Memory*> mColorImages;
	std::vector<const bgfx::Memory*> mDepthImages;
	std::vector<Joint[JointType_Count]> mJointData;

	const unsigned int mChunkSize;
	const unsigned int mNumChunks;

	unsigned int mCurrentChunk = 0;
	unsigned int mCurrentImageInChunk = 0;

public:
	ReconstructionState(unsigned int chunkSize, unsigned int numChunks)
		: mNumChunks(numChunks)
		, mChunkSize(chunkSize)
		, mColorImages(chunkSize*numChunks)
		, mDepthImages(chunkSize*numChunks)
		, mJointData(chunkSize*numChunks)
	{

	}

	void reset()
	{
		mCurrentChunk = 0;
		mCurrentImageInChunk = 0;
	}

	void feed(const bgfx::Memory* colorImage, const bgfx::Memory* depthImage, Joint joints[JointType_Count])
	{
		if(hasEnoughData())
		{
			return;
		}

		const auto index = mCurrentChunk*mChunkSize + mCurrentImageInChunk;
		mColorImages[index] = colorImage;
		mDepthImages[index] = depthImage;
		for (int i = 0; i < JointType_Count;i++) 
		{
			mJointData[index][i] = joints[i];
		}
		
		if (++mCurrentImageInChunk >= mChunkSize)
		{
			mCurrentImageInChunk = 0;
			mCurrentChunk++;
		}
	}

	float targetAngle() const 
	{
		return static_cast<int>(2*mCurrentChunk * 100. / mNumChunks)%100  - 50.; // 100 degree from the front and 100 degree from the back...
	}

	bool hasEnoughData() const
	{
		return mCurrentChunk == mNumChunks;
	}

	std::vector<const bgfx::Memory*> reconstructAvatar(KinectDataProvider* KDP)
	{
		
		auto CM = KDP->CoordinateMapper();
		std::vector<const bgfx::Memory*> SuperresDepthImages(mNumChunks);

		for (int curDepthImageIdx = 0; curDepthImageIdx < 1; curDepthImageIdx++)
		{
			std::unique_ptr<DepthSpacePoint[]> depthPoints(new DepthSpacePoint[KDP->ColorDataWidth()*KDP->ColorDataHeight()], std::default_delete<DepthSpacePoint[]>());
			auto depthDataRaw = reinterpret_cast<uint16_t*>(mDepthImages[curDepthImageIdx]->data);
			CM->MapColorFrameToDepthSpace(mDepthImages[curDepthImageIdx]->size / sizeof(uint16_t), depthDataRaw, KDP->ColorDataWidth()*KDP->ColorDataHeight(), depthPoints.get());

			// superresolution
			Eigen::MatrixXf A(3, KDP->ColorDataWidth()*KDP->ColorDataHeight());

			constexpr float errorBound = 0.005;
			float error = 0;
			SuperresDepthImages[curDepthImageIdx] = bgfx::alloc(KDP->ColorDataWidth()*KDP->ColorDataHeight()*sizeof(uint16_t));
			// make an initial guess
			auto lastIter = bgfx::copy(SuperresDepthImages[curDepthImageIdx]->data, SuperresDepthImages[curDepthImageIdx]->size);		
			for (int pos = 0; pos < KDP->ColorDataWidth()*KDP->ColorDataHeight(); pos++)
			{
				auto sourcePos = static_cast<int>(depthPoints[pos].Y*KDP->DepthDataWidth() + depthPoints[pos].X);
				SuperresDepthImages[curDepthImageIdx]->data[pos] = mDepthImages[curDepthImageIdx]->data[sourcePos];
			}
			do // approximate with gauss-seidel
			{
				error = 0;
				for (int y = 1;y < KDP->ColorDataHeight() - 1; y++)
				{
					for (int x = 1;x < KDP->ColorDataWidth() - 1; x++)
					{

					}
				}
				//SuperresDepthImages[curDepthImageIdx]->data
			} 
			while (error > errorBound);
		}
		
		return SuperresDepthImages;
	}

	std::shared_ptr<RenderImage> calcDepthDeviation() const
	{
		auto DepthDeviation = std::make_shared<RenderImage>(512, 424);
		for (int x = 0; x < DepthDeviation->width(); x++)
		{
			for (int y = 0; y < DepthDeviation->height(); y++)
			{
				auto reference = reinterpret_cast<UINT16*>((*mDepthImages.begin())->data);
				auto lastInChunk = mDepthImages.begin();
				std::advance(lastInChunk, mChunkSize);

				auto dev = std::accumulate(++mDepthImages.begin(), lastInChunk, 0, [&x, &y, &reference](int acc, const bgfx::Memory* image) {return acc + reference[512 * y + x] - reinterpret_cast<UINT16*>(image->data)[512 * y + x];}) / mDepthImages.size();

				DepthDeviation->writePixel(x, y, dev / 16);
			}
		}
		DepthDeviation->update();
		return DepthDeviation;
	}
};



#include <algorithm>
#include <numeric>
int _main_(int _argc, char** _argv)
{
	const char* const appTitle = "KinectAvatar";

	Args args(_argc, _argv);

	uint32_t windowWidth = 1600;
	uint32_t windowHeight = 900;
	uint32_t debug = BGFX_DEBUG_TEXT;
	uint32_t reset = BGFX_RESET_VSYNC;

	bgfx::init(args.m_type, args.m_pciId);
	bgfx::reset(windowWidth, windowHeight, reset);

	// Enable debug text.
	bgfx::setDebug(debug);

	// Set view 0 clear state.
	bgfx::setViewClear(0
		, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH
		, 0x303030ff
		, 1.0f
		, 0
		);

	const bgfx::Caps* caps = bgfx::getCaps();
	const bool computeSupported = !!(caps->supported & BGFX_CAPS_COMPUTE);

	if (computeSupported)
	{
		KinectDataProvider KDP;
		
		bool runningReconstruction = false;
		ReconstructionState reconstruction(10, 22);
		
		//assumption: tracking space is 10x10x8 (x,y,z) meters
		constexpr unsigned int kinectSkeletonSpaceX = 10;
		constexpr unsigned int kinectSkeletonSpaceY = 10;
		constexpr unsigned int kinectSkeletonSpaceZ = 8;

		constexpr unsigned int triplanarWidth = 500;
		constexpr unsigned int triplanarHeight = 500;

		float bodyAngle = 0.;
		int lastTrackedBody = -1;

		RenderImage xPlane(triplanarWidth, triplanarHeight),
			yPlane(triplanarWidth, triplanarHeight),
			zPlane(triplanarWidth, triplanarHeight);

		bgfx::TextureHandle colorTexture = bgfx::createTexture2D(KDP.ColorDataWidth(), KDP.ColorDataHeight(), false, 1, bgfx::TextureFormat::RGBA8, BGFX_TEXTURE_NONE);
		bgfx::TextureHandle depthRampTexture = bgfx::createTexture2D(KDP.DepthDataWidth(), KDP.DepthDataHeight(), false, 1, bgfx::TextureFormat::RGBA8, BGFX_TEXTURE_NONE);

		// Imgui.
		imguiCreate();
		// general settings
		bool showDepthBuffer = false;
		bool showColorImage = true;
		bool showTriplanarView = false;
		// color image settings
		bool showSkeletons = false;
		bool showJoints = false;

		entry::MouseState mouseState;
		while (!entry::processEvents(windowWidth, windowHeight, debug, reset, &mouseState))
		{
			try
			{
				// Check for new data.
				if (KDP.RefreshData())
				{
					//clear buffers
					xPlane.clear();
					yPlane.clear();
					zPlane.clear();

					// acquire latest data
					auto LatestColorImage = KDP.LatestColorData();
					auto ColorBuffer = bgfx::copy(LatestColorImage->data, LatestColorImage->size);

					for (int bodyIndex = 0; bodyIndex < BODY_COUNT; ++bodyIndex)
					{
						IBody* body = KDP.LatestBodyData(bodyIndex);
						BOOLEAN isTracked = false;
						if (SUCCEEDED(body->get_IsTracked(&isTracked)) && isTracked)
						{
							lastTrackedBody = bodyIndex;
							Joint joints[JointType_Count];
							body->GetJoints(JointType_Count, joints);

							{
								//http://www.ilikebigbits.com/blog/2015/3/2/plane-from-points
								Plane bodyPlane;
								std::vector<Vector3> trackedPoints;
								for (auto joint : joints)
								{
									if (joint.TrackingState == TrackingState_Tracked)
									{
										Vector3 lv(joint.Position);
										trackedPoints.push_back(lv);
									}
								}
								auto sum = std::accumulate(trackedPoints.begin(), trackedPoints.end(), Vector3{ 0, 0, 0 });
								auto centroid = sum*(1. / trackedPoints.size());
								// covariance matrix
								float xx = 0, xy = 0, xz = 0,
									yy = 0, yz = 0, zz = 0;
								for (auto pos : trackedPoints)
								{
									auto r = pos - centroid;
									xx += r.x * r.x;
									xy += r.x * r.y;
									xz += r.x * r.z;
									yy += r.y * r.y;
									yz += r.y * r.z;
									zz += r.z * r.z;
								}
								// determinant
								const auto detx = yy*zz - yz*yz;
								const auto dety = xx*zz - xz*xz;
								const auto detz = yy*xx - xy*xy;
								const auto detmax = std::max({ detx, dety, detz });
								if (detmax <= 0) // not a plane
								{
									continue;
								}
								if (detx == detmax)
								{
									Vector3 normal = { 1., (xz*yz - xy*zz) / detx , (xy*yz - xz*yy) / detx };
									normal.normalize();
									if (normal.z < 0) {
										normal.z *= -1;
										normal.x *= -1;
									}
									bodyPlane = Plane(normal, centroid);
								}
								else if (dety == detmax)
								{
									Vector3 normal = { (yz*xz - xy*zz) / dety, 1. , (xy*xz - yz*xx) / dety };
									normal.normalize();
									bodyPlane = Plane(normal, centroid);
								}
								else if (detz == detmax)
								{
									Vector3 normal = { (yz*xy - xz*yy) / detz, (xz*xy - yz*xx) / detz, 1. };
									normal.normalize();
									bodyPlane = Plane(normal, centroid);
								}
								bodyPlane.b = 0.0f;
								Plane xyPlane({ 0.,0.,-1. }, { 0.,0.,0. });
								//bodyAngle = angle(xyPlane, bodyPlane)*57.296f;
								bodyAngle = atan2(bodyPlane.a, bodyPlane.c)*57.296f;
								if (bodyAngle > 270.f)
								{
									bodyAngle -= 360;
								}
								else if (bodyAngle > 90.f)
								{
									bodyAngle -= 180;
								}
							}

							if (showJoints)
							{
								auto CoordinateMapper = KDP.CoordinateMapper();
								ColorSpacePoint fragment;
								for (int jointIndex = 0; jointIndex < JointType_Count; ++jointIndex)
								{
									const int radius = 10; // radius in pixels
									CoordinateMapper->MapCameraPointToColorSpace(joints[jointIndex].Position, &fragment);
									auto nearestPixel = [&KDP](const int X, const int Y) -> int {return sizeof(RGBQUAD)*(Y*KDP.ColorDataWidth() + X);};
									// draw circle
									for (int y = std::max<int>(std::round(fragment.Y) - radius, 0); y < std::min<int>(KDP.ColorDataHeight(), std::round(fragment.Y) + radius); y++)
									{
										for (int x = std::max<int>(std::round(fragment.X) - radius, 0); x < std::min<int>(KDP.ColorDataWidth(), std::round(fragment.X) + radius); x++)
										{
											int pixel = nearestPixel(x, y);
											ColorBuffer->data[pixel] = 255;
											ColorBuffer->data[pixel + 1] = 0;
											ColorBuffer->data[pixel + 2] = 0;
											ColorBuffer->data[pixel + 3] = 255;
										}
									}
								}
							}

							if (showTriplanarView)
							{
								constexpr int size = 4;
								for (auto joint : joints)
								{
									if (joint.TrackingState == TrackingState_Tracked)
									{
										{
											constexpr float ScaleFactorX = triplanarWidth / kinectSkeletonSpaceX;
											constexpr float ScaleFactorY = triplanarHeight / kinectSkeletonSpaceY;
											const auto xScaled = (2.*joint.Position.X + 5.)*ScaleFactorX;
											const auto yScaled = (-2.*joint.Position.Y + 5.)*ScaleFactorY;
											DrawQuad(zPlane, size, xScaled, yScaled);
										}
										{
											constexpr float ScaleFactorX = triplanarWidth / kinectSkeletonSpaceX;
											constexpr float ScaleFactorZ = triplanarHeight / kinectSkeletonSpaceZ;
											const auto xScaled = (-2.*joint.Position.X + 5.)*ScaleFactorX;
											const auto zScaled = joint.Position.Z*ScaleFactorZ;
											DrawQuad(yPlane, size, xScaled, zScaled);
										}
										{
											constexpr float ScaleFactorY = triplanarWidth / kinectSkeletonSpaceY;
											constexpr float ScaleFactorZ = triplanarHeight / kinectSkeletonSpaceZ;
											const auto yScaled = (-2.*joint.Position.Y + 5.)*ScaleFactorY;
											const auto zScaled = joint.Position.Z*ScaleFactorZ;
											DrawQuad(xPlane, size, zScaled, yScaled);
										}
									}
								}
							}

							xPlane.update();
							yPlane.update();
							zPlane.update();
						}
					}
					bgfx::updateTexture2D(colorTexture, 0, 0, 0, 0, KDP.ColorDataWidth(), KDP.ColorDataHeight(), ColorBuffer);


					auto LatestDepthBuffer = KDP.LatestDepthData();
					// Convert D16 to RGBA8 (grey ramp)
					auto FixedDepthBuffer = reinterpret_cast<UINT16*>(LatestDepthBuffer->data);
					const bgfx::Memory* DepthImage = bgfx::alloc(KDP.DepthDataWidth()*KDP.DepthDataHeight() * 4);
					for (int i = 0;i < LatestDepthBuffer->size / 2; i++)
					{
						for (int j = 0;j < 3; j++)
						{
							DepthImage->data[4 * i + j] = FixedDepthBuffer[i] / 16;
						}
						DepthImage->data[4 * i + 3] = 255;
					}
					bgfx::updateTexture2D(depthRampTexture, 0, 0, 0, 0, KDP.DepthDataWidth(), KDP.DepthDataHeight(), DepthImage);


					if (runningReconstruction)
					{
						if (reconstruction.targetAngle() - 5. < bodyAngle && bodyAngle < reconstruction.targetAngle() + 5.)
						{
							auto body = KDP.LatestBodyData(lastTrackedBody);
							Joint joints[JointType_Count];
							body->GetJoints(JointType_Count, joints);
							reconstruction.feed(bgfx::copy(LatestColorImage->data, LatestColorImage->size)
								, bgfx::copy(LatestDepthBuffer->data, LatestDepthBuffer->size)
								, joints);
							static bool doOnce = true;
							if (reconstruction.hasEnoughData() && doOnce)
							{
								reconstruction.reconstructAvatar(&KDP);
								doOnce = false;
							}
						}
					}
				}
			}
			catch(...)
			{ }
			bgfx::setViewRect(0, 0, 0, windowWidth, windowHeight);

			bgfx::dbgTextClear();
			bgfx::dbgTextPrintf(0, 1, 0x4f, appTitle);

			imguiBeginFrame(mouseState.m_mx
				, mouseState.m_my
				, (mouseState.m_buttons[entry::MouseButton::Left] ? IMGUI_MBUT_LEFT : 0)
				| (mouseState.m_buttons[entry::MouseButton::Right] ? IMGUI_MBUT_RIGHT : 0)
				| (mouseState.m_buttons[entry::MouseButton::Middle] ? IMGUI_MBUT_MIDDLE : 0)
				, mouseState.m_mz
				, windowWidth
				, windowHeight
				);
				
				ImGui::Begin("General Settings");
					ImGui::Checkbox("Show Color Image", &showColorImage);
					ImGui::Checkbox("Show Depth Buffer", &showDepthBuffer);
					ImGui::Separator();
					ImGui::Checkbox("Show Triplanar View", &showTriplanarView);
					ImGui::Separator();
				ImGui::End();

				ImGui::Begin("Reconstruction Panel");
					ImGui::BeginGroup();
						ImGui::Value("Current angle", bodyAngle);

						ImGui::SameLine();
						ImGui::Value("Target angle", reconstruction.targetAngle());
					ImGui::EndGroup();
					ImGui::ProgressBar(0.);
					ImGui::BeginGroup();
						if (ImGui::Button("Start reconstruction"))
						{
							runningReconstruction = true;
						}
						ImGui::SameLine();
						if (ImGui::Button("Cancel reconstruction"))
						{
							runningReconstruction = false;
							reconstruction.reset();
						}
					ImGui::EndGroup(); 
				ImGui::End();

				if (showDepthBuffer)
				{
					ImGui::SetNextWindowSizeConstraints(ImVec2(400, 320), ImVec2(KDP.DepthDataWidth(), KDP.DepthDataHeight()));
					ImGui::Begin("Depth Buffer");
						ImGui::Image(depthRampTexture, ImGui::GetContentRegionAvail());
					ImGui::End();
				}

				if (showColorImage)
				{
					ImGui::SetNextWindowSizeConstraints(ImVec2(400, 320), ImVec2(KDP.ColorDataWidth(), KDP.ColorDataHeight()));
					ImGui::Begin("Color Image");
						ImGui::Image(colorTexture, ImGui::GetContentRegionAvail());
					ImGui::End();

					ImGui::Begin("Color Image Settings");
						ImGui::Checkbox("Show Skeletons", &showSkeletons);
						ImGui::Checkbox("Show Joints", &showJoints);
					ImGui::End();
				}

				if (showTriplanarView)
				{
					ImGui::SetNextWindowSizeConstraints(ImVec2(400, 400), ImVec2(1000, 1000));
					ImGui::Begin("Triplanar View");
						auto Region = ImGui::GetContentRegionAvail();
						ImGui::Columns(2, 0, false);
						ImGui::Image(zPlane.handle(), { Region.x / 2, Region.y / 2});
						ImGui::Image(yPlane.handle(), { Region.x / 2, Region.y / 2});
						ImGui::NextColumn();
						ImGui::Image(xPlane.handle(), { Region.x / 2, Region.y / 2});
					ImGui::End();
				}

				if (reconstruction.hasEnoughData())
				{
					ImGui::SetNextWindowSizeConstraints(ImVec2(400, 320), ImVec2(KDP.ColorDataWidth(), KDP.ColorDataHeight()));
					//ImGui::Begin("Depth image deviation");
					//	ImGui::Image(reconstruction.getDepthDeviation()->handle(), ImGui::GetContentRegionAvail());
					//ImGui::End();
				}
				
			imguiEndFrame();

			bgfx::touch(0);
			bgfx::frame();
		}

		imguiDestroy();
	}
	else
	{
		int64_t timeOffset = bx::getHPCounter();

		entry::MouseState mouseState;
		while (!entry::processEvents(windowWidth, windowHeight, debug, reset, &mouseState))
		{
			int64_t now = bx::getHPCounter();
			float time = (float)((now - timeOffset) / double(bx::getHPFrequency()));

			bgfx::setViewRect(0, 0, 0, windowWidth, windowHeight);

			bgfx::dbgTextClear();
			bgfx::dbgTextPrintf(0, 1, 0x4f, appTitle);

			bool blink = uint32_t(time*3.0f) & 1;
			bgfx::dbgTextPrintf(0, 5, blink ? 0x1f : 0x01, " Compute is not supported by GPU. ");

			bgfx::touch(0);
			bgfx::frame();
		}
	}

	return 0;
}