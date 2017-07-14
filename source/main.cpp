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

	const bgfx::Memory* indexBuffer = nullptr;
	int indexBufferWidth = 0;
	int indexBufferHeight = 0;

public:
	KinectDataProvider()
	{
		constexpr auto SourceTypes = FrameSourceTypes_Color | FrameSourceTypes_Depth | FrameSourceTypes_Body | FrameSourceTypes_BodyIndex;

		// Grab reader
		HRESULT hr = GetDefaultKinectSensor(sensor.address());
		if (sensor.isset() || FAILED(hr))
		{
			throw std::runtime_error("Failed to acquire sensor.");
		}
		if (FAILED(sensor->Open()))
		{
			throw std::runtime_error("Failed to open sensor.");
		}
		if (FAILED(sensor->OpenMultiSourceFrameReader(SourceTypes, multiSourceReader.address())))
		{
			throw std::runtime_error("Failed to open reader.");
		}
		if (FAILED(sensor->get_CoordinateMapper(coordinateMapper.address())))
		{
			throw std::runtime_error("Failed to grab coordinate mapper.");
		}

		// Create buffers
		ComWrapper<IMultiSourceFrame> frame;
		while ((hr = multiSourceReader->AcquireLatestFrame(frame.address())) == E_PENDING) {} // fix to acquire buffer
		if (FAILED(hr))
		{
			throw std::runtime_error("Failed to grab initial frame.");
		}

		// depth
		{
			ComWrapper<IDepthFrameReference> depthFrameRef;
			if (FAILED(frame->get_DepthFrameReference(depthFrameRef.address())))
			{
				throw std::runtime_error("Failed to grab initial depth frame reference.");
			}
			ComWrapper<IDepthFrame> depthFrame;
			if (FAILED(depthFrameRef->AcquireFrame(depthFrame.address())))
			{
				throw std::runtime_error("Failed to grab initial depth frame.");
			}
			ComWrapper<IFrameDescription> depthFrameDesc;
			if (FAILED(depthFrame->get_FrameDescription(depthFrameDesc.address())))
			{
				throw std::runtime_error("Failed to grab depth frame description.");
			}
			if (FAILED(depthFrameDesc->get_Width(&depthBufferWidth)))
			{
				throw std::runtime_error("Failed to grab depth frame description.");
			}
			if (FAILED(depthFrameDesc->get_Height(&depthBufferHeight)))
			{
				throw std::runtime_error("Failed to grab depth frame description.");
			}
			depthBuffer = bgfx::alloc(depthBufferWidth*depthBufferHeight*sizeof(unsigned short));
		}
		
		// color
		{
			ComWrapper<IColorFrameReference> colorFrameRef;
			if (FAILED(frame->get_ColorFrameReference(colorFrameRef.address())))
			{
				throw std::runtime_error("Failed to grab initial color frame reference.");
			}
			ComWrapper<IColorFrame> colorFrame;
			if (FAILED(colorFrameRef->AcquireFrame(colorFrame.address())))
			{
				throw std::runtime_error("Failed to grab initial color frame.");
			}
			ComWrapper<IFrameDescription> colorFrameDesc;
			if (FAILED(colorFrame->get_FrameDescription(colorFrameDesc.address())))
			{
				throw std::runtime_error("Failed to grab color frame description.");
			}
			if (FAILED(colorFrameDesc->get_Width(&colorBufferWidth)))
			{
				throw std::runtime_error("Failed to grab color frame description.");
			}
			if (FAILED(colorFrameDesc->get_Height(&colorBufferHeight)))
			{
				throw std::runtime_error("Failed to grab color frame description.");
			}
			colorBuffer = bgfx::alloc(colorBufferWidth*colorBufferHeight*sizeof(RGBQUAD));
		}

		// index
		{
			ComWrapper<IBodyIndexFrameReference> indexFrameRef;
			if (FAILED(frame->get_BodyIndexFrameReference(indexFrameRef.address())))
			{
				throw std::runtime_error("Failed to grab initial body index frame reference.");
			}
			ComWrapper<IBodyIndexFrame> indexFrame;
			if (FAILED(indexFrameRef->AcquireFrame(indexFrame.address())))
			{
				throw std::runtime_error("Failed to grab initial color frame.");
			}
			ComWrapper<IFrameDescription> indexFrameDesc;
			if (FAILED(indexFrame->get_FrameDescription(indexFrameDesc.address())))
			{
				throw std::runtime_error("Failed to grab index frame description.");
			}
			if (FAILED(indexFrameDesc->get_Width(&indexBufferWidth)))
			{
				throw std::runtime_error("Failed to grab color frame description.");
			}
			if (FAILED(indexFrameDesc->get_Height(&indexBufferHeight)))
			{
				throw std::runtime_error("Failed to grab color frame description.");
			}
			indexBuffer = bgfx::alloc(indexBufferWidth*indexBufferHeight*sizeof(BYTE));
		}
	}

	bool RefreshData()
	{
		ComWrapper<IMultiSourceFrame> frame;
		if (FAILED(multiSourceReader->AcquireLatestFrame(frame.address())))
		{
			return false;
		}
		// depth
		{
			ComWrapper<IDepthFrameReference> depthFrameRef;
			if (FAILED(frame->get_DepthFrameReference(depthFrameRef.address())))
			{
				throw std::runtime_error("Failed to grab depth frame reference.");
			}
			ComWrapper<IDepthFrame> depthFrame;
			if (FAILED(depthFrameRef->AcquireFrame(depthFrame.address())))
			{
				throw std::runtime_error("Failed to grab depth frame.");
			}
			if (FAILED(depthFrame->CopyFrameDataToArray(depthBuffer->size / 2, reinterpret_cast<UINT16*>(depthBuffer->data))))
			{
				throw std::runtime_error("Failed to copy depth data.");
			}
		}
		// color
		{
			ComWrapper<IColorFrameReference> colorFrameRef;
			if (FAILED(frame->get_ColorFrameReference(colorFrameRef.address())))
			{
				throw std::runtime_error("Failed to grab color frame reference.");
			}
			ComWrapper<IColorFrame> colorFrame;
			if (FAILED(colorFrameRef->AcquireFrame(colorFrame.address())))
			{
				throw std::runtime_error("Failed to grab color frame.");
			}
			if (FAILED(colorFrame->CopyConvertedFrameDataToArray(colorBuffer->size, reinterpret_cast<BYTE*>(colorBuffer->data), ColorImageFormat_Rgba)))
			{
				throw std::runtime_error("Failed to copy color data.");
			}
		}
		

		// body
		{
			ComWrapper<IBodyFrameReference> bodyFrameRef;
			if (FAILED(frame->get_BodyFrameReference(bodyFrameRef.address())))
			{
				throw std::runtime_error("Failed to grab body frame reference.");
			}
			ComWrapper<IBodyFrame> bodyFrame;
			if (FAILED(bodyFrameRef->AcquireFrame(bodyFrame.address())))
			{
				throw std::runtime_error("Failed to grab body frame.");
			}
			if (FAILED(bodyFrame->GetAndRefreshBodyData(_countof(bodies), bodies)))
			{ 
				throw std::runtime_error("Failed to grab body data."); 
			}
		}


		// index
		{
			ComWrapper<IBodyIndexFrameReference> indexFrameRef;
			if (FAILED(frame->get_BodyIndexFrameReference(indexFrameRef.address())))
			{
				throw std::runtime_error("Failed to grab body index frame reference.");
			}
			ComWrapper<IBodyIndexFrame> indexFrame;
			if (FAILED(indexFrameRef->AcquireFrame(indexFrame.address())))
			{
				throw std::runtime_error("Failed to grab body index frame.");
			}
			if (FAILED(indexFrame->CopyFrameDataToArray(indexBuffer->size, reinterpret_cast<BYTE*>(indexBuffer->data))))
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

	const int IndexDataWidth() const
	{
		return this->indexBufferWidth;
	}

	const int IndexDataHeight() const
	{
		return this->indexBufferHeight;
	}

	const bgfx::Memory* LatestIndexData() const
	{
		return this->indexBuffer;
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

#include <Eigen/Core>
#include <Eigen/Eigenvalues> 

#include <iostream>

#include <pcl/common/projection_matrix.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/surface/poisson.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>

#include <pcl/visualization/cloud_viewer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <opencv2/opencv.hpp>

float angleForChunk(const int const current, const int total)
{
	return static_cast<int>(2 * current * 100. / total) % 100 - 50.;
}

//! @TODO refactor shit
class ReconstructionState
{
private:
	std::vector<Bgfx2DMemoryHelper<RGBQUAD>> mColorImages;
	std::vector<Bgfx2DMemoryHelper<uint16_t>> mDepthImages;
	std::vector<Bgfx2DMemoryHelper<uint8_t>> mIndexImages;
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
		, mIndexImages(chunkSize*numChunks)
		, mJointData(chunkSize*numChunks)
	{

	}

	void reset()
	{
		mCurrentChunk = 0;
		mCurrentImageInChunk = 0;
	}

	void feed(Bgfx2DMemoryHelper<RGBQUAD> colorImage, Bgfx2DMemoryHelper<uint16_t> depthImage, Bgfx2DMemoryHelper<uint8_t> indexImage, Joint joints[JointType_Count])
	{
		if(hasEnoughData())
		{
			return;
		}

		const auto index = mCurrentChunk*mChunkSize + mCurrentImageInChunk;
		mColorImages[index] = colorImage;
		mDepthImages[index] = depthImage;
		mIndexImages[index] = indexImage;
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
		return angleForChunk(mCurrentChunk, mNumChunks); // 100 degree from the front and 100 degree from the back...
	}

	bool hasEnoughData() const
	{
		return mCurrentChunk == mNumChunks;
	}

	std::shared_ptr<RenderImage> reconstructAvatar(KinectDataProvider* KDP)
	{
		auto CM = KDP->CoordinateMapper();
		std::vector<Bgfx2DMemoryHelper<uint16_t>> superresDepthImages(mNumChunks);

		std::vector<Bgfx2DMemoryHelper<float>> W(mNumChunks); // buffer for Wk
		for (int i = 0;i < mNumChunks;i++)
		{
			W[i] = Bgfx2DMemoryHelper<float>(KDP->ColorDataWidth(), KDP->ColorDataHeight());
		}


		for (int curDepthImageIdx = 0; curDepthImageIdx < superresDepthImages.size(); curDepthImageIdx++)
		{
			std::unique_ptr<DepthSpacePoint[]> depthPoints(new DepthSpacePoint[KDP->ColorDataWidth()*KDP->ColorDataHeight()], std::default_delete<DepthSpacePoint[]>());
			CM->MapColorFrameToDepthSpace(mDepthImages[curDepthImageIdx].width()*mDepthImages[curDepthImageIdx].height(), mDepthImages[curDepthImageIdx].raw(), KDP->ColorDataWidth()*KDP->ColorDataHeight(), depthPoints.get());

			//@TODO local registration via optical flow

			// superresolution
			constexpr float errorBound = 0.005;
			constexpr float gamma = 0.8;
			constexpr float eigenvalueThreshold = 5.0;

			superresDepthImages[curDepthImageIdx] = Bgfx2DMemoryHelper<uint16_t>(KDP->ColorDataWidth(), KDP->ColorDataHeight());
			// make an initial guess
			for (int y = 0; y < KDP->ColorDataHeight(); y++)
			{
				for (int x = 0; x < KDP->ColorDataWidth(); x++)
				{
					auto dp = depthPoints[y*KDP->ColorDataWidth() + x];
					if (isinf(dp.X) || isinf(dp.Y))
					{
						superresDepthImages[curDepthImageIdx].write(x, y, 0);
					}
					else
					{
						superresDepthImages[curDepthImageIdx].write(x, y, mDepthImages[mChunkSize*curDepthImageIdx + mChunkSize / 2].read(dp.X, dp.Y) );
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
						W[k].write(x, y, (0xFFFFFF - (*reinterpret_cast<uint32_t*>(&mColorImages[curDepthImageIdx*mChunkSize + k].read(x, y)) - sum / mChunkSize)) / 0xFFFFFF);
					}
				}
			}

			std::cout << "Starting approximation " << curDepthImageIdx << "/" << superresDepthImages.size() << std::endl;
			// approximation with gauss-seidel
			float error = 0;
			do
			{
				error = 0;
				auto prevData = superresDepthImages[curDepthImageIdx].clone();
				for (int y = 2;y < KDP->ColorDataHeight() - 2; y++)
				{
					for (int x = 2;x < KDP->ColorDataWidth() - 2; x++)
					{

						//first two loops give k
						auto sumWk = 0.;
						auto b = 0.;
						for (int k = 0;k < mNumChunks;k++)
						{
							sumWk += W[k].read(x, y);

							auto pos = depthPoints[y*KDP->ColorDataWidth() + x];
							if (!isinf(pos.X) && !isinf(pos.Y))
							{
								b += W[k].read(x, y)*mDepthImages[curDepthImageIdx].read(pos.X, pos.Y);
							}
						}
						int sumNeighbourhood = 0;
						// upper-left neighbourhood
						sumNeighbourhood += superresDepthImages[curDepthImageIdx].read(x - 2, y - 2) / sqrt(8);
						sumNeighbourhood += superresDepthImages[curDepthImageIdx].read(x - 1, y - 2) / sqrt(5);
						sumNeighbourhood += superresDepthImages[curDepthImageIdx].read(x - 0, y - 2) / sqrt(4);
						sumNeighbourhood += superresDepthImages[curDepthImageIdx].read(x + 1, y - 2) / sqrt(5);
						sumNeighbourhood += superresDepthImages[curDepthImageIdx].read(x + 2, y - 2) / sqrt(8);
						sumNeighbourhood += superresDepthImages[curDepthImageIdx].read(x - 2, y - 1) / sqrt(2);
						sumNeighbourhood += superresDepthImages[curDepthImageIdx].read(x - 1, y - 1) / sqrt(2);
						sumNeighbourhood += superresDepthImages[curDepthImageIdx].read(x - 0, y - 1) / sqrt(1);
						sumNeighbourhood += superresDepthImages[curDepthImageIdx].read(x + 1, y - 1) / sqrt(2);
						sumNeighbourhood += superresDepthImages[curDepthImageIdx].read(x + 2, y - 1) / sqrt(5);
						sumNeighbourhood += superresDepthImages[curDepthImageIdx].read(x - 2, y - 0) / sqrt(4);
						sumNeighbourhood += superresDepthImages[curDepthImageIdx].read(x - 1, y - 0) / sqrt(1);
						// lower-right neighbourhood
						sumNeighbourhood += prevData.read(x + 1, y - 0) / sqrt(2);
						sumNeighbourhood += prevData.read(x + 2, y - 0) / sqrt(4);
						sumNeighbourhood += prevData.read(x - 2, y + 1) / sqrt(5);
						sumNeighbourhood += prevData.read(x - 1, y + 1) / sqrt(2);
						sumNeighbourhood += prevData.read(x - 0, y + 1) / sqrt(1);
						sumNeighbourhood += prevData.read(x + 1, y + 1) / sqrt(2);
						sumNeighbourhood += prevData.read(x + 2, y + 1) / sqrt(5);
						sumNeighbourhood += prevData.read(x - 2, y + 2) / sqrt(8);
						sumNeighbourhood += prevData.read(x - 1, y + 2) / sqrt(5);
						sumNeighbourhood += prevData.read(x - 0, y + 2) / sqrt(4);
						sumNeighbourhood += prevData.read(x + 1, y + 2) / sqrt(5);
						sumNeighbourhood += prevData.read(x + 2, y + 2) / sqrt(8);

						//SuperresDepthImages[curDepthImageIdx].write(x, y, (b + gamma*sumNeighbourhood) / (sumWk + 48 * gamma));
						superresDepthImages[curDepthImageIdx].write(x, y, (b + gamma*sumNeighbourhood) / (sumWk + 48 * gamma));
					}
				}
			} while (error > errorBound);

			std::cout << "Finished approximation " << curDepthImageIdx+1 << "/" << superresDepthImages.size() << std::endl;
		
		}

		// segmentation
		std::cout << "Starting segmentation" << std::endl;
		{
			std::unique_ptr<DepthSpacePoint[]> depthPoints(new DepthSpacePoint[KDP->ColorDataWidth()*KDP->ColorDataHeight()], std::default_delete<DepthSpacePoint[]>());
			for (int curDepthImageIdx = 0; curDepthImageIdx < superresDepthImages.size(); curDepthImageIdx++)
			{
				const auto index = curDepthImageIdx*mChunkSize + mChunkSize / 2;
				CM->MapColorFrameToDepthSpace(mDepthImages[index].width()*mDepthImages[index].height(), mDepthImages[index].raw(), KDP->ColorDataWidth()*KDP->ColorDataHeight(), depthPoints.get());

				// create silhouette
				cv::Mat cvImage(KDP->ColorDataHeight(), KDP->ColorDataWidth(), CV_16UC1);
				for (int y = 0;y < KDP->ColorDataHeight(); y++)
				{
					for (int x = 0;x < KDP->ColorDataWidth(); x++)
					{
						auto dp = depthPoints[y*KDP->ColorDataWidth() + x];
						cvImage.at<uint16_t>(y, x) = 0;
						for (auto &indexImage : mIndexImages)
						{
							if (dp.X > 0 && indexImage.read(dp.X, dp.Y) < 6)
							{
								cvImage.at<uint16_t>(y, x) = superresDepthImages[curDepthImageIdx].read(x, y);
							}
						}
						//cvImage.at<uint16_t>(y, x) = superresDepthImages[i].read(x, y);
					}
				}

				// save superres images to file
				std::string filename = std::to_string(curDepthImageIdx);
				filename += "-angle";
				filename += std::to_string(angleForChunk(curDepthImageIdx, mNumChunks));
				filename += ".png";
				try {
					cv::imwrite(filename.c_str(), cvImage);
					std::cout << "Successfully saved superres data to" << filename << std::endl;
				}
				catch (std::runtime_error& ex) {
					std::cout << "Exception converting image to PNG format: " << ex.what() << std::endl;
					continue;
				}
			}
		}
		std::cout << "Finished segmentation" << std::endl;

		// generate point clouds
		std::cout << "Starting point cloud generation" << std::endl;
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pointClouds(superresDepthImages.size());
		{
			std::unique_ptr<DepthSpacePoint[]> depthPoints(new DepthSpacePoint[KDP->ColorDataWidth()*KDP->ColorDataHeight()], std::default_delete<DepthSpacePoint[]>());
			std::unique_ptr<CameraSpacePoint[]> cameraPoints(new CameraSpacePoint[KDP->ColorDataWidth()*KDP->ColorDataHeight()], std::default_delete<CameraSpacePoint[]>());
			for (int curDepthImageIdx = 0; curDepthImageIdx < superresDepthImages.size(); curDepthImageIdx++)
			{
				const auto index = curDepthImageIdx*mChunkSize + mChunkSize / 2;
				pointClouds[curDepthImageIdx] = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
				CM->MapColorFrameToDepthSpace(mDepthImages[index].width()*mDepthImages[index].height(), mDepthImages[index].raw(), KDP->ColorDataWidth()*KDP->ColorDataHeight(), depthPoints.get());
				CM->MapDepthFrameToCameraSpace(superresDepthImages[curDepthImageIdx].width()*superresDepthImages[curDepthImageIdx].height(), superresDepthImages[curDepthImageIdx].raw(), superresDepthImages[curDepthImageIdx].width()*superresDepthImages[curDepthImageIdx].height(), cameraPoints.get());
			
				for (int y = 0;y < KDP->ColorDataHeight(); y++)
				{
					for (int x = 0;x < KDP->ColorDataWidth(); x++)
					{
						auto dp = depthPoints[y*KDP->ColorDataWidth() + x];
						//for (auto &indexImage : mIndexImages) 
						{
							if (!isinf(dp.X) /* && indexImage.read(dp.X, dp.Y) != 0xff */)
							{
								auto color = mColorImages[index].read(x, y);
								pcl::PointXYZRGB p;
								p.x = x;
								p.y = y;
								p.z = superresDepthImages[curDepthImageIdx].read(x, y);
								//p.z = mDepthImages[index].read(dp.X, dp.Y);
								p.r = color.rgbRed;
								p.g = color.rgbBlue;
								p.b = color.rgbGreen;
								pointClouds[curDepthImageIdx]->push_back(p);
							}
						}
					}
				}
				
				/*
				for (int pointIndex = 0; pointIndex < superresDepthImages[0].width()*superresDepthImages[0].height(); pointIndex++)
				{
					CameraSpacePoint p = cameraPoints[pointIndex];
					if (p.Z <= 5.) // only map close points
					{
						pointClouds[i]->push_back({ p.X, p.Y, p.Z });
					}
					
				}
				*/
			}
		}
		std::cout << "Finished point cloud generation" << std::endl;

		pcl::visualization::PCLVisualizer viewer("PCL Viewer");
		viewer.setBackgroundColor(0.0, 0.0, 0.5);
		viewer.addPointCloud<pcl::PointXYZRGB>(pointClouds[0]);
		while (!viewer.wasStopped())
		{
			viewer.spinOnce();
		}
		/*
		std::cout << "Starting normal estimation" << std::endl;
		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> neomp;
		neomp.setNumberOfThreads(8);
		neomp.setRadiusSearch(0.1);
		neomp.setInputCloud(pointClouds[0]);
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*pointClouds[0], centroid);
		neomp.setViewPoint(centroid[0], centroid[1], centroid[2]);
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());
		neomp.compute(*cloud_normals);
		for (size_t i = 0; i < cloud_normals->size(); ++i) {
			cloud_normals->points[i].normal_x *= -1;
			cloud_normals->points[i].normal_y *= -1;
			cloud_normals->points[i].normal_z *= -1;
		}
		std::cout << "Finished normal estimation" << std::endl;
		*/
		/*
		std::cout << "Starting normal estimation" << std::endl;
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
		ne.setMaxDepthChangeFactor(0.02f);
		ne.setNormalSmoothingSize(10.0f);
		ne.setInputCloud(cloud);
		ne.compute(*normals);
		std::cout << "Finished normal estimation" << std::endl;
		*/
		/*
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals(new pcl::PointCloud<pcl::PointNormal>());
		pcl::concatenateFields(*pointClouds[0], *cloud_normals, *cloud_smoothed_normals);

		std::cout << "Starting poisson reconstruction" << std::endl;
		pcl::Poisson<pcl::PointNormal> poisson;
		poisson.setDepth(9);
		poisson.setInputCloud(cloud_smoothed_normals);
		poisson.setInputCloud(pointClouds[0]);
		pcl::PolygonMesh mesh;
		poisson.reconstruct(mesh);
		std::cout << "Finished poisson reconstruction" << std::endl;

		pcl::io::savePLYFile("testmesh.ply", mesh);
		*/
		return std::make_shared<RenderImage>(KDP->ColorDataWidth(), KDP->ColorDataHeight());
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
								, Bgfx2DMemoryHelper<uint8_t>(KDP.IndexDataWidth(), KDP.IndexDataHeight(), KDP.LatestIndexData()).clone()
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