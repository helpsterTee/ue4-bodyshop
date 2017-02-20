#include "common.h"
#include "bgfx_utils.h"
#include "imgui/imgui.h"
#include "camera.h"
#include <bgfx/bgfx.h>
#include "Kinect.h"
#include <stdexcept>

template<class Interface>
static inline void SafeRelease(Interface *&interfaceToRelease)
{
	if (interfaceToRelease != nullptr) {
		interfaceToRelease->Release();
		interfaceToRelease = nullptr;
	}
}

// Interface to manage COM classes in an exception-safe manner
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


// Synchronous data provider
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
		hr = sensor->OpenMultiSourceFrameReader(FrameSourceTypes_Color | FrameSourceTypes_Depth | FrameSourceTypes_Body, multiSourceReader.address());
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


#include <algorithm>
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

		bgfx::TextureHandle colorTexture = bgfx::createTexture2D(KDP.ColorDataWidth(), KDP.ColorDataHeight(), false, 1, bgfx::TextureFormat::RGBA8, BGFX_TEXTURE_NONE);
		bgfx::TextureHandle depthRampTexture = bgfx::createTexture2D(KDP.DepthDataWidth(), KDP.DepthDataHeight(), false, 1, bgfx::TextureFormat::RGBA8, BGFX_TEXTURE_NONE);

		// Imgui.
		imguiCreate();
		// general settings
		bool showDepthBuffer = false;
		bool showColorImage = true;
		// color image settings
		bool showSkeletons = false;

		entry::MouseState mouseState;
		while (!entry::processEvents(windowWidth, windowHeight, debug, reset, &mouseState))
		{
			// Check for new data.
			if (KDP.RefreshData())
			{
				auto LatestColorImage = KDP.LatestColorData();
				auto ColorBuffer = bgfx::copy(LatestColorImage->data, LatestColorImage->size);
				if (showSkeletons)
				{
					for (int bodyIndex = 0; bodyIndex < BODY_COUNT; ++bodyIndex)
					{
						IBody* body = KDP.LatestBodyData(bodyIndex);
						BOOLEAN isTracked = false;
						if (SUCCEEDED(body->get_IsTracked(&isTracked)) && isTracked)
						{
							auto CoordinateMapper = KDP.CoordinateMapper();
							Joint joints[JointType_Count];
							body->GetJoints(JointType_Count, joints);
							ColorSpacePoint fragment;
							for (int jointIndex = 0; jointIndex < JointType_Count; ++jointIndex)
							{
								int radius = 5; // radius in pixels
								CoordinateMapper->MapCameraPointToColorSpace(joints[jointIndex].Position, &fragment);
								auto nearestPixel = [&KDP](int X, int Y) -> int {return sizeof(RGBQUAD)*(Y*KDP.ColorDataWidth() + X);};
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
					}
				}
				bgfx::updateTexture2D(colorTexture, 0, 0, 0, 0, KDP.ColorDataWidth(), KDP.ColorDataHeight(), ColorBuffer);


				auto LatestDepthBuffer = KDP.LatestDepthData();
				// Convert D16 to RGBA8 (grey ramp)
				auto FixedDepthBuffer = reinterpret_cast<UINT16*>(LatestDepthBuffer->data);
				const bgfx::Memory* DepthImage = bgfx::alloc(KDP.DepthDataWidth()*KDP.DepthDataHeight() * 4);
				for (int i = 0;i < LatestDepthBuffer->size/2; i++)
				{
					for (int j = 0;j < 4; j++)
					{
						DepthImage->data[4 * i + j] = FixedDepthBuffer[i] / 16;
					}
				}
				bgfx::updateTexture2D(depthRampTexture, 0, 0, 0,0 ,KDP.DepthDataWidth(), KDP.DepthDataHeight(), DepthImage);
			}

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
					ImGui::End();
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