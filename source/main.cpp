#include <common.h>
#include <bgfx_utils.h>
#include <imgui/imgui.h>
#include <camera.h>
#include <opencv2/core.hpp>
#include <bgfx/bgfx.h>
#include <Kinect.h>
#include <stdexcept>
#include <cmath>


#include "util.hpp"

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



#include "render_image.hpp"



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
#include <algorithm>

#include <opencv2/superres.hpp>
#include <eigen/Core>

#include <iostream>
class ReconstructionState
{
private:
	std::vector<Bgfx2DMemoryHelper<RGBQUAD>> mColorImages;
	std::vector<Bgfx2DMemoryHelper<uint16_t>> mDepthImages;
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

	void feed(Bgfx2DMemoryHelper<RGBQUAD> colorImage, Bgfx2DMemoryHelper<uint16_t> depthImage, Joint joints[JointType_Count])
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

	std::shared_ptr<RenderImage> reconstructAvatar(KinectDataProvider* KDP)
	{
		auto CM = KDP->CoordinateMapper();
		std::vector<Bgfx2DMemoryHelper<uint16_t>> SuperresDepthImages(mNumChunks);

		std::vector<Bgfx2DMemoryHelper<float>> W(mNumChunks); // buffer for Wk
		for (int i = 0;i < mNumChunks;i++)
		{
			W[i] = Bgfx2DMemoryHelper<float>(KDP->ColorDataWidth(), KDP->ColorDataHeight());
		}
		

		for (int curDepthImageIdx = 0; curDepthImageIdx < 1; curDepthImageIdx++)
		{
			std::unique_ptr<DepthSpacePoint[]> depthPoints(new DepthSpacePoint[KDP->ColorDataWidth()*KDP->ColorDataHeight()], std::default_delete<DepthSpacePoint[]>());
			CM->MapColorFrameToDepthSpace(mDepthImages[curDepthImageIdx].width()*mDepthImages[curDepthImageIdx].height(), mDepthImages[curDepthImageIdx].raw(), KDP->ColorDataWidth()*KDP->ColorDataHeight(), depthPoints.get());

			// superresolution
			constexpr float errorBound = 0.005;
			constexpr float gamma = 0.8;

			SuperresDepthImages[curDepthImageIdx] = Bgfx2DMemoryHelper<uint16_t>(KDP->ColorDataWidth(), KDP->ColorDataHeight());
			// make an initial guess
			for (int y = 0; y < KDP->ColorDataHeight(); y++)
			{
				for (int x = 0; x < KDP->ColorDataWidth(); x++)
				{
					auto dp = depthPoints[y*KDP->ColorDataWidth() + x];
					if (isinf(dp.X) || isinf(dp.Y))
					{
						SuperresDepthImages[curDepthImageIdx].write(x, y, 0);
					}
					else
					{
						SuperresDepthImages[curDepthImageIdx].write(x, y, mDepthImages[mChunkSize*curDepthImageIdx+mChunkSize/2].read(dp.X, dp.Y)/16);
					}
				}				
			}
			// prepare W
			for (int k = 0;k < mNumChunks; k++)
			{
				for (int y = 1;y < KDP->ColorDataHeight() - 1; y++)
				{
					for (int x = 1;x < KDP->ColorDataWidth() - 1; x++)
					{
						int sum = 0;
						for (int ks = 0;ks < mChunkSize;ks++)
						{
							sum += *reinterpret_cast<uint32_t*>(&mColorImages[curDepthImageIdx*mChunkSize + k].read(x, y));
						}
						W[k].write(x, y, (0xFFFFFF - (*reinterpret_cast<uint32_t*>(&mColorImages[curDepthImageIdx*mChunkSize + k].read(x, y)) - sum / mChunkSize))/ 0xFFFFFF);
					}
				}
			}
			//std::cout << "Starting approximation " << curDepthImageIdx << std::endl;
			// approximation with gauss-seidel
			float error = 0;
			do 
			{
				error = 0;
				auto prevData = SuperresDepthImages[curDepthImageIdx].clone();
				for (int y = 1;y < KDP->ColorDataHeight() - 1; y++)
				{
					for (int x = 1;x < KDP->ColorDataWidth() - 1; x++)
					{
						//first two loops give k
						//auto superresData = reinterpret_cast<uint16_t*>(SuperresDepthImages[curDepthImageIdx]->data);
						//auto previousSuperresData = reinterpret_cast<uint16_t*>(last->data);
						auto sumWk = 0.;
						auto b = 0.;
						for (int k = 0;k < mNumChunks;k++)
						{
							sumWk += W[k].read(x, y);

							auto pos = depthPoints[y*KDP->ColorDataWidth() + x];
							if (!isinf(pos.X) && !isinf(pos.Y))
							{
								b += mDepthImages[curDepthImageIdx].read(pos.X,pos.Y);
							}
						}
						auto xLeft = SuperresDepthImages[curDepthImageIdx].read(x - 1, y);
						auto xUpper = SuperresDepthImages[curDepthImageIdx].read(x, y - 1);
						auto xRight = prevData.read(x + 1, y);
						auto xLower = prevData.read(x, y + 1);
						SuperresDepthImages[curDepthImageIdx].write(x, y, (b+gamma*(xLeft+xUpper+xRight+xLower))/(sumWk+8*gamma));
					}
				}
			} 
			while (error > errorBound);
		}
		
		auto Result = std::make_shared<RenderImage>(KDP->ColorDataWidth(), KDP->ColorDataHeight());

		std::unique_ptr<DepthSpacePoint[]> depthPoints(new DepthSpacePoint[KDP->ColorDataWidth()*KDP->ColorDataHeight()], std::default_delete<DepthSpacePoint[]>());
		CM->MapColorFrameToDepthSpace(mDepthImages[0].width()*mDepthImages[0].height(), mDepthImages[0].raw(), KDP->ColorDataWidth()*KDP->ColorDataHeight(), depthPoints.get());
		
		for (int y = 0;y < KDP->ColorDataHeight(); y++)
		{
			for (int x = 0;x < KDP->ColorDataWidth(); x++)
			{
				auto current = depthPoints[y*KDP->ColorDataWidth() + x];
				if (!isinf(current.X) && current.X >= 0)
				{
					///@TODO FIX CONVERSION ERROR!!!!!!!!!!!!!!!!!!!!!!!! Somewhere...
					//auto val = mDepthImages[0].read(current.X, current.Y) / 16;
					//SuperresDepthImages[0].write(x, y, mDepthImages[0].read(current.X, current.Y));
					//Result->writePixel(x, y, mDepthImages[0].read(x, y) / 16);
					//Result->writePixel(x, y, mDepthImages[0].read(current.X, current.Y) / 16);
					Result->writePixel(x, y, SuperresDepthImages[0].read(x, y));
				}
				//Result->writePixel(x, y, SuperresDepthImages[0].read(x, y));
			}
		}

		Result->update();
		return Result;
	}

	std::shared_ptr<RenderImage> calcDepthDeviation() const
	{
		auto DepthDeviation = std::make_shared<RenderImage>(mDepthImages[0].width(), mDepthImages[0].height());
		for (int x = 0; x < DepthDeviation->width(); x++)
		{
			for (int y = 0; y < DepthDeviation->height(); y++)
			{
				auto reference = (mDepthImages.begin())->raw();
				auto lastInChunk = mDepthImages.begin();
				std::advance(lastInChunk, mChunkSize);

				auto dev = std::accumulate(++mDepthImages.begin(), lastInChunk, 0, [&x, &y, &reference](int acc, Bgfx2DMemoryHelper<uint16_t> image) {return acc + reference[512 * y + x] - image.read(x,y);}) / mDepthImages.size();

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
		ReconstructionState reconstruction(10, 1);
		
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

		std::shared_ptr<RenderImage> SResRes;

		//bgfx::TextureHandle colorTexture = bgfx::createTexture2D(KDP.ColorDataWidth(), KDP.ColorDataHeight(), false, 1, bgfx::TextureFormat::RGBA8, BGFX_TEXTURE_NONE);
		auto ColorImage = RenderImage(KDP.ColorDataWidth(), KDP.ColorDataHeight());
		auto DepthRampImage = RenderImage(KDP.DepthDataWidth(), KDP.DepthDataHeight());

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
					auto LatestColorImage = Bgfx2DMemoryHelper<RGBQUAD>(KDP.ColorDataWidth(), KDP.ColorDataHeight(), KDP.LatestColorData());
					for (int y = 0; y < LatestColorImage.height(); y++)
					{
						for (int x = 0; x < LatestColorImage.width(); x++)
						{
							auto val = LatestColorImage.read(x, y);
							ColorImage.writePixel(x, y, val.rgbBlue, val.rgbGreen, val.rgbRed, 255);
						}
					}
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
									// draw circle
									for (int y = std::max<int>(std::round(fragment.Y) - radius, 0); y < std::min<int>(KDP.ColorDataHeight(), std::round(fragment.Y) + radius); y++)
									{
										for (int x = std::max<int>(std::round(fragment.X) - radius, 0); x < std::min<int>(KDP.ColorDataWidth(), std::round(fragment.X) + radius); x++)
										{
											ColorImage.writePixel(x, y, 255, 0, 0, 255);
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
					ColorImage.update();
					

					auto LatestDepthBuffer = Bgfx2DMemoryHelper<uint16_t>(KDP.DepthDataWidth(), KDP.DepthDataHeight(), KDP.LatestDepthData());
					// Convert D16 to RGBA8 (grey ramp)
					for (int y = 0; y < LatestDepthBuffer.height(); y++)
					{
						for (int x = 0;x < LatestDepthBuffer.width(); x++)
						{
							DepthRampImage.writePixel(x, y, LatestDepthBuffer.read(x,y) / 16 );
						}
					}
					DepthRampImage.update();
					
					 
					if (runningReconstruction)
					{
						if (reconstruction.targetAngle() - 5. < bodyAngle && bodyAngle < reconstruction.targetAngle() + 5.)
						{
							auto body = KDP.LatestBodyData(lastTrackedBody);
							Joint joints[JointType_Count];
							body->GetJoints(JointType_Count, joints);
							reconstruction.feed(LatestColorImage.clone()
								, LatestDepthBuffer.clone()
								, joints);
							static bool doOnce = true;
							if (reconstruction.hasEnoughData() && doOnce)
							{
								SResRes = reconstruction.reconstructAvatar(&KDP);
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
						ImGui::Image(DepthRampImage.handle(), ImGui::GetContentRegionAvail());
					ImGui::End();
				}

				if (showColorImage)
				{
					ImGui::SetNextWindowSizeConstraints(ImVec2(400, 320), ImVec2(KDP.ColorDataWidth(), KDP.ColorDataHeight()));
					ImGui::Begin("Color Image");
						ImGui::Image(ColorImage.handle(), ImGui::GetContentRegionAvail());
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

				if (SResRes)
				{
					ImGui::SetNextWindowSizeConstraints(ImVec2(400, 400), ImVec2(KDP.ColorDataWidth(), KDP.ColorDataHeight()));
					ImGui::Begin("RESULT");
						ImGui::Image(SResRes->handle(), ImGui::GetContentRegionAvail());
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