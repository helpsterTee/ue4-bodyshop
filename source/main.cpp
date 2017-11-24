#include <common.h>
#include <bgfx_utils.h>
#include <imgui/imgui.h>
#include <camera.h>
#include <opencv2/core.hpp>
#include <bgfx/bgfx.h>
#include <Kinect.h>
#include <stdexcept>
#include <cmath>

// @todo replace....
#define LOGURU_IMPLEMENTATION 1
#define LOGURU_WITH_STREAMS 1
#include "loguru.hpp"

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

#include <omp.h> 

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>

#include <iostream>

#include <pcl/common/projection_matrix.h>
#include <pcl/common/geometry.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/surface/poisson.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/supervoxel_clustering.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/optflow.hpp>
#include <opencv2/video.hpp>

#include <opengm/graphicalmodel/graphicalmodel.hxx>
#include <opengm/operations/adder.hxx>
#include <opengm/operations/minimizer.hxx>
#include <opengm/inference/graphcut.hxx>
#include <opengm/inference/alphaexpansion.hxx>
#include <opengm/inference/auxiliary/minstcutboost.hxx>
#include <opengm/graphicalmodel/space/simplediscretespace.hxx>
#include <opengm/functions/potts.hxx>

constexpr int Nnear = 20;

float angleForChunk(const int const current, const int total)
{
	//return static_cast<int>(2 * current * 100. / total) % 100 - 50.;
	std::array<float, 3> angles = { -50., 0., 50. };
	assert(angles.max_size * 2 == total);
	return angles[current % 3];
}

template<typename T>
void copyToCvMat(Bgfx2DMemoryHelper<T>& internalImage, cv::Mat& cvImage)
{
	for (int y = 0; y < internalImage.height(); y++)
	{
		for (int x = 0; x < internalImage.width(); x++)
		{
			cvImage.at<T>(y, x) = internalImage.read(x, y);
		}
	}
}

void copyToCvMat(Bgfx2DMemoryHelper<RGBQUAD>& internalImage, cv::Mat& cvImage)
{
	for (int y = 0; y < internalImage.height(); y++)
	{
		for (int x = 0; x < internalImage.width(); x++)
		{
			cvImage.at<cv::Vec3b>(y, x)[0] = internalImage.read(x, y).rgbGreen;
			cvImage.at<cv::Vec3b>(y, x)[1] = internalImage.read(x, y).rgbBlue;
			cvImage.at<cv::Vec3b>(y, x)[2] = internalImage.read(x, y).rgbRed;
		}
	}
}

template<typename PointT>
uint32_t getHighestLabel(boost::shared_ptr<pcl::PointCloud<PointT>> pointCloud)
{
	uint32_t highestLabel = 0;
	for (const auto &pt : (*pointCloud))
	{
		highestLabel = std::max(highestLabel, pt.label);
	}
	return highestLabel;
}


template<typename PointT> 
std::vector<pcl::PointIndices> getLabelIndices(boost::shared_ptr<pcl::PointCloud<PointT>> pointCloud)
{
	std::vector<pcl::PointIndices> labelVector(getHighestLabel(pointCloud));
	for (int i = 0; i < pointCloud->size(); i++)
	{
		const auto &pt = (*pointCloud)[i];
		highestLabel = std::max(highestLabel, pt.label);
		labelVector[pt.label].push_back(i);
	}
	return labelVector;
}


#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
//! @TODO refactor shit
class ReconstructionState
{
private:
	std::vector<Bgfx2DMemoryHelper<RGBQUAD>> mColorImages;
	std::vector<Bgfx2DMemoryHelper<uint16_t>> mDepthImages;
	std::vector<Bgfx2DMemoryHelper<uint8_t>> mIndexImages;
	std::vector<Joint[JointType_Count]> mJointData;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> mPointClouds;

	const unsigned int mChunkSize;
	const unsigned int mNumChunks;

	unsigned int mCurrentChunk = 0;
	unsigned int mCurrentImageInChunk = 0;


public:
	ReconstructionState(const unsigned int chunkSize, const unsigned int numChunks)
		: mNumChunks(numChunks)
		, mChunkSize(chunkSize)
		, mColorImages(chunkSize*numChunks)
		, mDepthImages(chunkSize*numChunks)
		, mIndexImages(chunkSize*numChunks)
		, mJointData(chunkSize*numChunks)
		, mPointClouds(numChunks)
	{
		for (int curChunkIdx = 0; curChunkIdx < numChunks; curChunkIdx++)
		{
			mPointClouds[curChunkIdx] = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
		}
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

	std::unique_ptr<std::pair<cv::Point, cv::Point>[]> boundaries; ///@todo remove
	void constructPointClouds(KinectDataProvider* KDP)
	{
		auto CM = KDP->CoordinateMapper();

		// compute body hull
		std::unique_ptr<DepthSpacePoint[]> depthPoints(new DepthSpacePoint[KDP->ColorDataWidth()*KDP->ColorDataHeight()], std::default_delete<DepthSpacePoint[]>());
		std::unique_ptr<cv::Mat[]> bodyHull(new cv::Mat[mNumChunks], std::default_delete<cv::Mat[]>());
		boundaries = std::unique_ptr<std::pair<cv::Point, cv::Point>[]>(new std::pair<cv::Point, cv::Point>[mNumChunks], std::default_delete<std::pair<cv::Point, cv::Point>[]>());

		for (int curChunkIdx = 0; curChunkIdx < mNumChunks; curChunkIdx++)
		{
			// init
			bodyHull[curChunkIdx] = cv::Mat(KDP->ColorDataHeight(), KDP->ColorDataWidth(), CV_8U);
			for (int y = 0;y < KDP->ColorDataHeight(); y++)
			{
				for (int x = 0;x < KDP->ColorDataWidth(); x++)
				{
					bodyHull[curChunkIdx].at<byte>(y, x) = false;
				}
			}

			// body hull = union over all index images
			for (int indexImageIdx = curChunkIdx*mChunkSize; indexImageIdx < curChunkIdx*mChunkSize + mChunkSize; indexImageIdx++)
			{
				CM->MapColorFrameToDepthSpace(KDP->DepthDataWidth()*KDP->DepthDataHeight(), mDepthImages[indexImageIdx].raw(), KDP->ColorDataWidth()*KDP->ColorDataHeight(), depthPoints.get());

				for (int y = 0;y < KDP->ColorDataHeight(); y++)
				{
					for (int x = 0;x < KDP->ColorDataWidth(); x++)
					{
						// Check whenether the point belongs to the body
						auto dp = depthPoints[y*KDP->ColorDataWidth() + x];
						if (!isinf(dp.X) && mIndexImages[indexImageIdx].read(dp.X, dp.Y) != 0xff)
						{
							bodyHull[curChunkIdx].at<byte>(y, x) = true;
						}
					}
				}
			}

			// make it slightly bigger
			const auto erosion_size = 2;
			const auto dilatation_size = 4;
			cv::Mat erosionElement = cv::getStructuringElement(cv::MORPH_ELLIPSE,
				cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
				cv::Point(erosion_size, erosion_size));
			cv::Mat dilatationElement = cv::getStructuringElement(cv::MORPH_ELLIPSE,
				cv::Size(2 * erosion_size + 1, 2 * dilatation_size + 1),
				cv::Point(erosion_size, erosion_size));
			cv::erode(bodyHull[curChunkIdx], bodyHull[curChunkIdx], erosionElement);
			cv::dilate(bodyHull[curChunkIdx], bodyHull[curChunkIdx], dilatationElement);

			// compute bounding box
			boundaries[curChunkIdx].first = cv::Point(KDP->ColorDataHeight(), KDP->ColorDataWidth());
			boundaries[curChunkIdx].second = cv::Point(0, 0);
			for (int y = 0;y < KDP->ColorDataHeight(); y++)
			{
				for (int x = 0;x < KDP->ColorDataWidth(); x++)
				{
					if (bodyHull[curChunkIdx].at<byte>(y, x))
					{
						boundaries[curChunkIdx].first.x = std::min(boundaries[curChunkIdx].first.x, x);
						boundaries[curChunkIdx].first.y = std::min(boundaries[curChunkIdx].first.y, y);
						boundaries[curChunkIdx].second.x = std::max(boundaries[curChunkIdx].second.x, x);
						boundaries[curChunkIdx].second.y = std::max(boundaries[curChunkIdx].second.y, y);
					}
				}
			}

			// save to file
			std::string filename = "data/body_hull_";
			filename += std::to_string(curChunkIdx + 1);
			filename += ".png";
			try {
				cv::imwrite(filename.c_str(), bodyHull[curChunkIdx]);
				LOG_S(INFO) << "Successfully saved body hull data to " << filename;
			}
			catch (std::runtime_error& ex) {
				LOG_S(ERROR) << "Exception converting " << filename << " image to PNG format: " << ex.what();
				continue;
			}
		}

		// preprocess data
		std::vector<cv::Mat> superresDepthImages(mNumChunks);
		#pragma omp parallel for
		for (int curChunkIdx = 0; curChunkIdx < mNumChunks; curChunkIdx++)
		{
			const auto bbLengthX = boundaries[curChunkIdx].second.x - boundaries[curChunkIdx].first.x + 1;
			const auto bbLengthY = boundaries[curChunkIdx].second.y - boundaries[curChunkIdx].first.y + 1;

			CM->MapColorFrameToDepthSpace(KDP->DepthDataWidth()*KDP->DepthDataHeight(), mDepthImages[curChunkIdx].raw(), KDP->ColorDataWidth()*KDP->ColorDataHeight(), depthPoints.get());
			const auto middleImageIdx = curChunkIdx*mChunkSize + mChunkSize / 2.;

			// crop away unneeded data
			std::vector<cv::Mat> cvColorImages(mChunkSize);
			std::vector<cv::Mat> cvDepthImages(mChunkSize);
			for (int i = 0;i < mChunkSize;i++)
			{
				const auto seriesImageIdx = curChunkIdx*mChunkSize + i;
				CM->MapColorFrameToDepthSpace(KDP->DepthDataWidth()*KDP->DepthDataHeight(), mDepthImages[seriesImageIdx].raw(), KDP->ColorDataWidth()*KDP->ColorDataHeight(), depthPoints.get());

				cvColorImages[i] = cv::Mat(bbLengthY, bbLengthX, CV_8UC3);
				cvDepthImages[i] = cv::Mat(bbLengthY, bbLengthX, CV_16U);
				for (int y = boundaries[curChunkIdx].first.y; y <= boundaries[curChunkIdx].second.y; y++)
				{
					for (int x = boundaries[curChunkIdx].first.x; x <= boundaries[curChunkIdx].second.x; x++)
					{
						const auto xcv = x - boundaries[curChunkIdx].first.x;
						const auto ycv = y - boundaries[curChunkIdx].first.y;
						auto dp = depthPoints[y*KDP->ColorDataWidth() + x];

						cvColorImages[i].at<cv::Vec3b>(ycv, xcv)[0] = 0;
						cvColorImages[i].at<cv::Vec3b>(ycv, xcv)[1] = 0;
						cvColorImages[i].at<cv::Vec3b>(ycv, xcv)[2] = 0;
						cvDepthImages[i].at<uint16_t>(ycv, xcv) = 0;

						//if (bodyHull[curChunkIdx].at<byte>(y, x))
						{
							auto color = mColorImages[seriesImageIdx].read(x, y);
							cvColorImages[i].at<cv::Vec3b>(ycv, xcv)[0] = color.rgbBlue;
							cvColorImages[i].at<cv::Vec3b>(ycv, xcv)[1] = color.rgbGreen;
							cvColorImages[i].at<cv::Vec3b>(ycv, xcv)[2] = color.rgbRed;
							if (!isinf(dp.X))
							{
								cvDepthImages[i].at<uint16_t>(ycv, xcv) = mDepthImages[seriesImageIdx].read(dp.X, dp.Y);
							}
						}
					}
				}
				{
					std::string filename = "data/raw_color_";
					filename += std::to_string(seriesImageIdx + 1);
					filename += ".png";
					try {
						cv::imwrite(filename.c_str(), cvColorImages[i]);
						LOG_S(INFO) << "Successfully saved raw color data to " << filename;
					}
					catch (std::runtime_error& ex) {
						LOG_S(ERROR) << "Exception converting " << filename << " image to PNG format: " << ex.what();
						continue;
					}
				}
				{
					std::string filename = "data/raw_depth_";
					filename += std::to_string(seriesImageIdx + 1);
					filename += ".png";
					try {
						cv::imwrite(filename.c_str(), cvDepthImages[i]);
						LOG_S(INFO) << "Successfully saved raw depth data to " << filename << std::endl;
					}
					catch (std::runtime_error& ex) {
						LOG_S(ERROR) << "Exception converting " << filename << " image to PNG format: " << ex.what() << std::endl;
						continue;
					}
				}
			}

			/*
			//remove the background
			///@TODO OpenCV BackgroundSubtractor
			for (int y = 0;y < KDP->ColorDataHeight(); y++)
			{
				for (int x = 0;x < KDP->ColorDataWidth(); x++)
				{
					if (!(boundaries[curChunkIdx].first.x <= x && x <= boundaries[curChunkIdx].second.x
						&& boundaries[curChunkIdx].first.y <= y && y <= boundaries[curChunkIdx].second.y))
					{
						for (int i = curChunkIdx*mChunkSize;i < (curChunkIdx + 1)*mChunkSize; i++)
						{
							mColorImages[i].write(x, y, { 0, 0, 0 });
						}
					}
				}
			}
			*/

			// compute alignments
			/*
			std::cout << "Aligning images in chunk " << curChunkIdx+1 << "/" << mNumChunks << std::endl;
			//cv::Mat cvMiddleColorImage = cv::Mat(KDP->ColorDataHeight(), KDP->ColorDataWidth(), CV_8UC3);
			//copyToCvMat(mColorImages[middleImageIdx], cvMiddleColorImage);

			//std::unique_ptr<cv::Mat[]> flowToMiddleFrames(new cv::Mat[mNumChunks], std::default_delete<cv::Mat[]>());
			cv::Ptr<cv::DenseOpticalFlow> optflow = cv::createOptFlow_DualTVL1();
			cv::Mat referenceImageGrey;
			cvtColor(cvColorImages[mChunkSize / 2], referenceImageGrey, CV_BGR2GRAY);
			for (int chunkImageIdx = 0; chunkImageIdx < mChunkSize; chunkImageIdx++)
			{
				if (chunkImageIdx == mChunkSize / 2) continue;

				cv::Mat greyImage;
				cvtColor(cvColorImages[chunkImageIdx], greyImage, CV_BGR2GRAY);

				std::cout << "Aligning" << chunkImageIdx + 1 << "/" << mChunkSize << std::endl;
				if (chunkImageIdx == mChunkSize / 2) { continue; } // skip self

				cv::Mat flow;
				optflow->calc(greyImage, referenceImageGrey, flow);
				{
					// save flow
					cv::Mat xy[2]; //X,Y
					cv::split(flow, xy);
					cv::Mat magnitude, angle;
					cv::cartToPolar(xy[0], xy[1], magnitude, angle, true);

					//translate magnitude to range [0;1]
					double mag_max;
					cv::minMaxLoc(magnitude, 0, &mag_max);
					magnitude.convertTo(magnitude, -1, 1.0 / mag_max);

					//build hsv image
					cv::Mat _hsv[3], hsv;
					_hsv[0] = angle;
					_hsv[1] = cv::Mat::ones(angle.size(), CV_32F);
					_hsv[2] = magnitude;
					cv::merge(_hsv, 3, hsv);

					//convert to BGR and show
					cv::Mat bgr;//CV_32FC3 matrix
					cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);

					std::string filename = "data/flow_";
					filename += std::to_string(curChunkIdx*mChunkSize+chunkImageIdx + 1);
					filename += ".png";
					try {
						cv::imwrite(filename.c_str(), bgr);
						std::cout << "Successfully saved flow data to " << filename << std::endl;
					}
					catch (std::runtime_error& ex) {
						std::cout << "Exception converting " << filename << " image to PNG format: " << ex.what() << std::endl;
						continue;
					}
				}


				cv::Mat mapx(flow.size(), CV_32FC1);
				cv::Mat mapy(flow.size(), CV_32FC1);
				for (int y = 0; y < flow.rows; ++y)
				{
					for (int x = 0; x < flow.cols; ++x)
					{
						cv::Point2f f = flow.at<cv::Point2f>(y, x);
						//map.at<cv::Point2f>(y, x) = cv::Point2f(x + f.x, y + f.y);
						mapx.at<float>(y, x) = x - f.x;
						mapy.at<float>(y, x) = y - f.y;
					}
				}

				{
					cv::Mat alignedImage;
					cv::remap(cvColorImages[chunkImageIdx], alignedImage, mapx, mapy, CV_INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
					cvColorImages[chunkImageIdx] = alignedImage;
					{
						std::string filename = "data/aligned_color_";
						filename += std::to_string(curChunkIdx*mChunkSize + chunkImageIdx + 1);
						filename += ".png";
						try {
							cv::imwrite(filename.c_str(), alignedImage);
							std::cout << "Successfully saved aligned color data to " << filename << std::endl;
						}
						catch (std::runtime_error& ex) {
							std::cout << "Exception converting " << filename << " image to PNG format: " << ex.what() << std::endl;
							continue;
						}
					}
				}
				{
					cv::Mat alignedImage;
					cv::remap(cvDepthImages[chunkImageIdx], alignedImage, mapx, mapy,   CV_INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
					cvDepthImages[chunkImageIdx] = alignedImage;
					{
						std::string filename = "data/aligned_depth_";
						filename += std::to_string(curChunkIdx*mChunkSize + chunkImageIdx + 1);
						filename += ".png";
						try {
							cv::imwrite(filename.c_str(), alignedImage);
							std::cout << "Successfully saved aligned depth data to " << filename << std::endl;
						}
						catch (std::runtime_error& ex) {
							std::cout << "Exception converting " << filename << " image to PNG format: " << ex.what() << std::endl;
							continue;
						}
					}
				}
			}
			*/

			/*
			std::shared_ptr<RenderImage> color0 = std::make_shared<RenderImage>(mColorImages[0].width(), mColorImages[0].height());
			for (int y = 0;y < mColorImages[0].height();y++)
			{
				for (int x = 0; x < mColorImages[0].width();x++)
				{
					auto rgb = cvColorImages[0].at<cv::Vec3b>(y, x);
					color0->writePixel(x, y, rgb[2], rgb[1], rgb[0]);
				}
			}
			color0->update();
			return color0;
			*/
			// superresolution
			{
				const auto bbLengthX = boundaries[curChunkIdx].second.x - boundaries[curChunkIdx].first.x + 1;
				const auto bbLengthY = boundaries[curChunkIdx].second.y - boundaries[curChunkIdx].first.y + 1;

				constexpr double errorBound = 0.005;
				constexpr int maxIter = 100;
				constexpr double gamma = 0.8;
				superresDepthImages[curChunkIdx] = cv::Mat(bbLengthY, bbLengthX, CV_16U);

				// make an initial guess
				for (int y = 0; y < bbLengthY; y++)
				{
					for (int x = 0; x < bbLengthX; x++)
					{
						auto depthValue = cvDepthImages[mChunkSize / 2].at<uint16_t>(y, x);
						superresDepthImages[curChunkIdx].at<uint16_t>(y, x) = depthValue;
					}
				}

				std::vector<cv::Mat> W(mChunkSize); // buffer for Wk
				for (int i = 0;i < mChunkSize;i++)
				{
					W[i] = cv::Mat(bbLengthY, bbLengthX, CV_32F);
				}
				for (int k = 0;k < mChunkSize; k++)
				{
					for (int y = 1;y < bbLengthY - 1; y++)
					{
						for (int x = 1;x < bbLengthX - 1; x++)
						{
							int sumR = 0;
							int sumG = 0;
							int sumB = 0;
							for (int ks = 0;ks < mChunkSize;ks++)
							{
								//auto color = cvColorImages[ks].at<cv::Vec3b>(y, x);
								//sum += color[0]<<2 + color[1]<<1 + color[2];
								auto color = mColorImages[curChunkIdx*mChunkSize + ks].read(x + boundaries[curChunkIdx].first.x, y + boundaries[curChunkIdx].first.y);
								sumR += color.rgbRed;
								sumG += color.rgbGreen;
								sumB += color.rgbBlue;
							}
							//W[k].at<float>(y, x) = (0xFFFFFF - (*reinterpret_cast<uint32_t*>(&mColorImages[curChunkIdx*mChunkSize + k].read(x + boundaries[curChunkIdx].first.x, y + boundaries[curChunkIdx].first.y)) - sum / mChunkSize)) / 0xFFFFFF;
							auto reference_color = mColorImages[curChunkIdx*mChunkSize + k].read(x + boundaries[curChunkIdx].first.x, y + boundaries[curChunkIdx].first.y);
							W[k].at<float>(y, x) = (255 - (reference_color.rgbRed - sumR/ mChunkSize))*(255 - (reference_color.rgbGreen - sumG/ mChunkSize))*(255 - (reference_color.rgbBlue - sumB/ mChunkSize));

							//auto color = cvColorImages[k].at<cv::Vec3b>(y, x);
							//W[k].at<float>(y, x) = (0xFFFFFF - ((color[0]<<2 + color[1]<<2 + color[2]) - sum / mChunkSize)) / 0xFFFFFF;
						}
					}
				}
				// approximation with gauss-seidel
				{
					LOG_SCOPE_F(INFO, "Starting approximation %i/%i.", curChunkIdx + 1, superresDepthImages.size());
					LOG_F(INFO, "%-20s | %-20s | %-20s", "iteration", "error", "iter_time");

					double error = 0;
					int iter = 0;
					do
					{
						auto iterStartTime = std::chrono::high_resolution_clock::now();
						error = 0;
						auto prevData = superresDepthImages[curChunkIdx].clone();
						for (int y = 2;y < bbLengthY - 2; y++)
						{
							for (int x = 2;x < bbLengthX - 2; x++)
							{

								//first two loops give k
								auto sumWk = 0.;
								auto b = 0.;
								for (int k = 0;k < mNumChunks;k++)
								{
									sumWk += W[k].at<float>(y, x);

									//auto pos = depthPoints[y*KDP->ColorDataWidth() + x];
									//if (!isinf(pos.X) && !isinf(pos.Y))
									{
										//b += W[k].at<float>(y, x)*mDepthImages[curChunkIdx].read(pos.X, pos.Y);
										b += W[k].at<float>(y, x)*cvDepthImages[k].at<uint16_t>(y, x);
									}
								}
								int sumNeighbourhood = 0;
								// upper-left neighbourhood
								sumNeighbourhood += superresDepthImages[curChunkIdx].at<uint16_t>(y - 2, x - 2) / sqrt(8);
								sumNeighbourhood += superresDepthImages[curChunkIdx].at<uint16_t>(y - 1, x - 2) / sqrt(5);
								sumNeighbourhood += superresDepthImages[curChunkIdx].at<uint16_t>(y - 0, x - 2) / sqrt(4);
								sumNeighbourhood += superresDepthImages[curChunkIdx].at<uint16_t>(y + 1, x - 2) / sqrt(5);
								sumNeighbourhood += superresDepthImages[curChunkIdx].at<uint16_t>(y + 2, x - 2) / sqrt(8);
								sumNeighbourhood += superresDepthImages[curChunkIdx].at<uint16_t>(y - 2, x - 1) / sqrt(2);
								sumNeighbourhood += superresDepthImages[curChunkIdx].at<uint16_t>(y - 1, x - 1) / sqrt(2);
								sumNeighbourhood += superresDepthImages[curChunkIdx].at<uint16_t>(y - 0, x - 1) / sqrt(1);
								sumNeighbourhood += superresDepthImages[curChunkIdx].at<uint16_t>(y + 1, x - 1) / sqrt(2);
								sumNeighbourhood += superresDepthImages[curChunkIdx].at<uint16_t>(y + 2, x - 1) / sqrt(5);
								sumNeighbourhood += superresDepthImages[curChunkIdx].at<uint16_t>(y - 2, x - 0) / sqrt(4);
								sumNeighbourhood += superresDepthImages[curChunkIdx].at<uint16_t>(y - 1, x - 0) / sqrt(1);
								// lower-right neighbourhood
								sumNeighbourhood += prevData.at<uint16_t>(y + 1, x - 0) / sqrt(2);
								sumNeighbourhood += prevData.at<uint16_t>(y + 2, x - 0) / sqrt(4);
								sumNeighbourhood += prevData.at<uint16_t>(y - 2, x + 1) / sqrt(5);
								sumNeighbourhood += prevData.at<uint16_t>(y - 1, x + 1) / sqrt(2);
								sumNeighbourhood += prevData.at<uint16_t>(y - 0, x + 1) / sqrt(1);
								sumNeighbourhood += prevData.at<uint16_t>(y + 1, x + 1) / sqrt(2);
								sumNeighbourhood += prevData.at<uint16_t>(y + 2, x + 1) / sqrt(5);
								sumNeighbourhood += prevData.at<uint16_t>(y - 2, x + 2) / sqrt(8);
								sumNeighbourhood += prevData.at<uint16_t>(y - 1, x + 2) / sqrt(5);
								sumNeighbourhood += prevData.at<uint16_t>(y - 0, x + 2) / sqrt(4);
								sumNeighbourhood += prevData.at<uint16_t>(y + 1, x + 2) / sqrt(5);
								sumNeighbourhood += prevData.at<uint16_t>(y + 2, x + 2) / sqrt(8);

								//SuperresDepthImages[curChunkIdx].write(x, y, (b + gamma*sumNeighbourhood) / (sumWk + 48 * gamma));
								superresDepthImages[curChunkIdx].at<uint16_t>(y, x) = (b + gamma*sumNeighbourhood) / (sumWk + 48 * gamma);
							}
						}
						auto iterEndTime = std::chrono::high_resolution_clock::now();
						auto iterationTime = std::chrono::duration_cast<std::chrono::milliseconds>(iterEndTime - iterStartTime).count();
						iter++;

						LOG_F(INFO, "%-20i | %-20d | %-20i ms", iter, error, iterationTime);

					} while (error > errorBound && iter < maxIter);
				}

				{
					std::string filename = "data/superres_depth";
					filename += std::to_string(curChunkIdx + 1);
					filename += ".png";
					try {
						cv::imwrite(filename.c_str(), superresDepthImages[curChunkIdx]);
						LOG_F(INFO, "Successfully saved superresolution data to %s", filename.c_str());
					}
					catch (std::runtime_error& ex) {
						LOG_F(INFO, "Exception converting %s image to PNG format: %s", filename.c_str(), ex.what());
						continue;
					}
				}
			}
		}

		// segmentation
		/*
		std::cout << "Starting segmentation" << std::endl;
		{
			std::unique_ptr<DepthSpacePoint[]> depthPoints(new DepthSpacePoint[KDP->ColorDataWidth()*KDP->ColorDataHeight()], std::default_delete<DepthSpacePoint[]>());
			for (int curChunkIdx = 0; curChunkIdx < mNumChunks; curChunkIdx++)
			{
				const auto aligneeImageIndex = curChunkIdx*mChunkSize + mChunkSize / 2;
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
		*/

		// generate point clouds
		{
			LOG_SCOPE_F(INFO, "Starting point cloud generation.");
			#pragma omp parallel for
			for (int curChunkIdx = 0; curChunkIdx < mNumChunks; curChunkIdx++)
			{
				const auto aligneeImageIndex = curChunkIdx*mChunkSize + mChunkSize / 2;
				const auto& curSuperresDepthImage = superresDepthImages[curChunkIdx];
				const auto& aligneeDepthImage = mDepthImages[aligneeImageIndex];
				const auto& aligneeColorImage = mColorImages[aligneeImageIndex];
				const auto& curBodyHull = bodyHull[curChunkIdx];

				// prefiltered save
				const auto bbLengthX = boundaries[curChunkIdx].second.x - boundaries[curChunkIdx].first.x + 1;
				const auto bbLengthY = boundaries[curChunkIdx].second.y - boundaries[curChunkIdx].first.y + 1;
				mPointClouds[curChunkIdx] = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
				for (int y = 0;y < bbLengthY; y++)
				{
					for (int x = 0;x < bbLengthX; x++)
					{
						if (curBodyHull.at<byte>(y + boundaries[curChunkIdx].first.y, x + boundaries[curChunkIdx].first.x))
						{
							auto color = aligneeColorImage.read(x + boundaries[curChunkIdx].first.x, y + boundaries[curChunkIdx].first.y);
							pcl::PointXYZRGB p;
							p.x = x;
							p.y = y;
							p.z = curSuperresDepthImage.at<uint16_t>(y, x);
							p.r = color.rgbBlue;
							p.g = color.rgbGreen;
							p.b = color.rgbRed;
							mPointClouds[curChunkIdx]->push_back(p);
						}
					}
				}
				{
					std::string filename = "data/point_cloud_";
					filename += std::to_string(curChunkIdx + 1);
					try {
						pcl::io::savePLYFile(filename + ".ply", *mPointClouds[curChunkIdx]);
						pcl::io::savePCDFile(filename + ".pcd", *mPointClouds[curChunkIdx]);
						LOG_S(INFO) << "Successfully saved point cloud to " << filename;
					}
					catch (std::runtime_error& ex) {
						LOG_S(ERROR) << "Exception converting point cloud to PLY format: " << ex.what();
						continue;
					}
				}

				// filter noise by reducing body hull
				cv::erode(curBodyHull, curBodyHull, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(12, 12)));

				// convert to point cloud
				for (int y = 0;y < bbLengthY; y++)
				{
					for (int x = 0;x < bbLengthX; x++)
					{
						if (curBodyHull.at<byte>(y + boundaries[curChunkIdx].first.y, x + boundaries[curChunkIdx].first.x))
						{
							// this shit does not work...
							//CM->MapDepthPointToCameraSpace(dp, curSuperresDepthImage.at<uint16_t>(y, x), &cp);
							auto color = aligneeColorImage.read(x + boundaries[curChunkIdx].first.x, y + boundaries[curChunkIdx].first.y);
							pcl::PointXYZRGB p;
							p.x = x;
							p.y = y;
							p.z = curSuperresDepthImage.at<uint16_t>(y, x);
							p.r = color.rgbBlue;
							p.g = color.rgbGreen;
							p.b = color.rgbRed;
							mPointClouds[curChunkIdx]->push_back(p);
						}
					}
				}

				// transform to camera space
				///@TODO transform the data manually
				for (size_t i = 0; i < mPointClouds[curChunkIdx]->points.size(); ++i)
				{
					auto& p = (*mPointClouds[curChunkIdx])[i];
					p.x = ((p.x + boundaries[curChunkIdx].first.x) / 1920. - 1.) * 3.6;
					p.y = (1. - (p.y + boundaries[curChunkIdx].first.y) / 1080.) * 2.;
					p.z = p.z / 800.;
				}
			}
		}

		preprocessPointClouds( );

		/*
		pcl::visualization::PCLVisualizer viewer("PCL Viewer");
		viewer.setBackgroundColor(0.0, 0.0, 0.5);
		viewer.addPointCloud<pcl::PointXYZRGB>(pointClouds[0]);
		while (!viewer.wasStopped())
		{
			viewer.spinOnce();
		}
		*/

		//pcl::io::savePLYFile("Point Cloud.ply",*pointClouds[0]);

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
	}


	void showPointClouds()
	{
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		viewer->setBackgroundColor(0, 0, 0);
		viewer->addCoordinateSystem(1.0);

		for (int curChunkIdx = 0; curChunkIdx < mNumChunks; curChunkIdx++)
		{
			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(mPointClouds[curChunkIdx]);
			viewer->addPointCloud<pcl::PointXYZRGB>(mPointClouds[curChunkIdx], rgb, "sample cloud" + std::to_string(curChunkIdx));
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
		}
		viewer->initCameraParameters();

		while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
	}


	void preprocessPointClouds()
	{
		LOG_SCOPE_FUNCTION(INFO);
		constexpr double radius = 0.035;
		constexpr int Nnear = 25;
		constexpr double clusterMaxDist = 0.04;


		#pragma omp parallel for
		for (int curChunkIdx = 0; curChunkIdx < mNumChunks; curChunkIdx++)
		{
			pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
			outrem.setRadiusSearch(radius);
			outrem.setMinNeighborsInRadius(Nnear);
			pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
			ec.setClusterTolerance(clusterMaxDist);
			ec.setMinClusterSize(1);

			LOG_SCOPE_F(INFO, "Preprocessing point cloud %i/%i", curChunkIdx + 1, mNumChunks);
			boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
			{
				/*
				LOG_SCOPE_F(INFO, "Remove radius outliers: radius=%f m, Nnear=%i", radius, Nnear);
				auto startTime = std::chrono::high_resolution_clock::now();
				outrem.setInputCloud(mPointClouds[curChunkIdx]);
				outrem.filter(*cloud_filtered);
				LOG_S(INFO) << "Took " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - startTime).count() << "ms";
				*/
				cloud_filtered = mPointClouds[curChunkIdx];
			}
			mPointClouds[curChunkIdx] = cloud_filtered;
			{
				LOG_SCOPE_F(INFO, "Extract biggest cluster with tolecance of %f m", clusterMaxDist);
				// build kdtree
				///@todo check whenever FLANN can be used here.
				pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
				kdtree->setInputCloud(mPointClouds[curChunkIdx]);
				// extract clusters
				std::vector<pcl::PointIndices> cluster_indices;
				ec.setMaxClusterSize(mPointClouds[curChunkIdx]->size());
				ec.setSearchMethod(kdtree);
				ec.setInputCloud(mPointClouds[curChunkIdx]);
				ec.extract(cluster_indices);

				pcl::PointIndices biggest_chunk = *cluster_indices.begin();
				for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
				{
					LOG_F(INFO, "Found cluster of size %i", it->indices.size());
					if (biggest_chunk.indices.size() > it->indices.size())
					{
						biggest_chunk = *it;
					}
				}
				pcl::ExtractIndices<pcl::PointXYZRGB> eifilter(true); // Initializing with true will allow us to extract the removed indices
				pcl::PointIndices::Ptr p_biggest_chunk( new pcl::PointIndices);
				*p_biggest_chunk = biggest_chunk;
				eifilter.setInputCloud(mPointClouds[curChunkIdx]);
				eifilter.setIndices(p_biggest_chunk);
				eifilter.filter(*cloud_filtered);
			}
			mPointClouds[curChunkIdx] = cloud_filtered;
			{
				std::string filename = "data/point_cloud_";
				filename += std::to_string(curChunkIdx + 1);
				filename += "_filtered_new";
				LOG_S(INFO) << "Saving filtered point cloud back to " << filename;
				try {
					pcl::io::savePLYFile(filename + ".ply", *mPointClouds[curChunkIdx]);
					pcl::io::savePCDFile(filename + ".pcd", *mPointClouds[curChunkIdx]);
					LOG_S(INFO) << "Successfully saved filtered point cloud";
				}
				catch (std::runtime_error& ex) {
					LOG_S(ERROR) << "Error saving point cloud: " << ex.what();
					continue;
				}
			}
		}
	}


	void downsamplePointClouds()
	{
		LOG_SCOPE_FUNCTION(INFO);
		constexpr double leafsize = 0.05;

		#pragma omp parallel for
		for (int curChunkIdx = 0; curChunkIdx < mNumChunks; curChunkIdx++)
		{
			pcl::VoxelGrid<pcl::PointXYZRGB> sor;
			sor.setLeafSize(leafsize, leafsize, leafsize);

			LOG_SCOPE_F(INFO, "Downsampling point cloud %i/%i", curChunkIdx + 1, mNumChunks);
			boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
			sor.setInputCloud(mPointClouds[curChunkIdx]);
			sor.filter(*cloud_filtered);
			mPointClouds[curChunkIdx] = cloud_filtered;
			{
				std::string filename = "data/point_cloud_";
				filename += std::to_string(curChunkIdx + 1);
				filename += "_downsampled";
				LOG_S(INFO) << "Saving filtered point cloud back to " << filename;
				try {
					pcl::io::savePLYFile(filename + ".ply", *mPointClouds[curChunkIdx]);
					pcl::io::savePCDFile(filename + ".pcd", *mPointClouds[curChunkIdx]);
					LOG_S(INFO) << "Successfully saved filtered point cloud";
				}
				catch (std::runtime_error& ex) {
					LOG_S(ERROR) << "Error saving point cloud: " << ex.what();
					continue;
				}
			}
		}
	}

	//! Cui et.al. (7)
	template<class TPOINT>
	double calculateVariance(const boost::shared_ptr<pcl::PointCloud<TPOINT>> f, const pcl::KdTreeFLANN<TPOINT>& gkdtree ) const
	{
		std::vector<int> pointIdxNKNSearch(Nnear);
		std::vector<float> pointNKNSquaredDistance(Nnear);
		double sum = 0.;
		for (int n = 0;n < f->size();n++)
		{
			auto numFoundPoints = gkdtree.nearestKSearch((*f)[n], Nnear, pointIdxNKNSearch, pointNKNSquaredDistance);
			for (int m = 0; m < numFoundPoints; m++)
			{
				sum += pointNKNSquaredDistance[m];
			}
		}
		return sum/(f->size()*Nnear);
	}

	// calculate the bayesian probability between m in f and m in g. Look at [2] from Cui et.al. for formula.
	template<class TPOINT>
	double calculatePOld(const TPOINT &yfn, const TPOINT &ygm, const pcl::KdTreeFLANN<TPOINT>& kdtree, const double variance)
	{
		std::vector<int> pointIdxNKNSearch(Nnear);
		std::vector<float> pointNKNSquaredDistance(Nnear);
		double sum = 0.;
		double minus2variance = -2 * variance;
		//for (int n = 0;n < kdtree.getInputCloud()->size();n++)
		{
			auto numFoundPoints = kdtree.nearestKSearch(ygm, Nnear, pointIdxNKNSearch, pointNKNSquaredDistance);
			for (int m = 0; m < numFoundPoints; m++)
			{
				sum += exp(pointNKNSquaredDistance[m]/minus2variance);
			}
		}
		double distSquared = (yfn.x - ygm.x)*(yfn.x - ygm.x) + (yfn.y - ygm.y)*(yfn.y - ygm.y) + (yfn.z - ygm.z)*(yfn.z - ygm.z);
		return exp(distSquared/minus2variance) / sum;
	}

	void reconstructAvatar()
	{
		LOG_SCOPE_FUNCTION(INFO);

		//  initial solution from capturing
		///@todo need something better
		std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBL>>> labeledPointClouds(mPointClouds.size());
		std::vector<std::vector<pcl::PointIndices>> cluster_indices(mPointClouds.size());

		uint32_t maxLabelPrev = 0;
		for (int f = 0; f < mPointClouds.size(); f++)
		{
			LOG_S(INFO) << "Initial labeling and transformation " << f + 1 << "/" << mPointClouds.size() << " with " << mPointClouds[f]->size() << " points";
			labeledPointClouds[f] = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBL>>();
			labeledPointClouds[f]->resize(mPointClouds[f]->size());

			// ------- initial registration --------
			///@todo implement something better (e.g. spin image matching)
			Eigen::Vector4f centroid;
			pcl::compute3DCentroid(*mPointClouds[f], centroid);
			float backsideRotation = (f >= mPointClouds.size() / 2) ? 0 : 180; //workaround for kinect 2 tracking (cannot detect backside properly)
			Eigen::AngleAxisf aa(-0.0175*(angleForChunk(f, mPointClouds.size()) + backsideRotation), Eigen::Vector3f(0., 1., 0.));
			Eigen::Vector3f trans = -1 * centroid.head<3>();
			Eigen::Affine3f trafo = aa*Eigen::Translation3f(trans);
			pcl::transformPointCloud(*mPointClouds[f], *mPointClouds[f], trafo);

			for (int idx = 0; idx < mPointClouds[f]->size(); idx++)
			{
				auto pt = mPointClouds[f]->at(idx);
				auto &ptLabeled = (*labeledPointClouds[f])[idx];
				ptLabeled.x = pt.x;
				ptLabeled.y = pt.y;
				ptLabeled.z = pt.z;
				ptLabeled.r = pt.r;
				ptLabeled.g = pt.g;
				ptLabeled.b = pt.b;
				ptLabeled.label = -1;
			}

			// -------- initial segmentation ----------
			///@todo factor out this code, parametrize it an implement an automatic optimal parameter calculation
			// use normal kernel
			auto g = [](double x)->double {
				return exp(-x / 2.) / 2.;
			};
			constexpr double bandwidth = 0.075;
			constexpr double bandwidth2 = bandwidth*bandwidth;
			constexpr int maxNumLabels = 50;
			boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> peakCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
			peakCloud->resize(mPointClouds[f]->size());
			
			#pragma omp parallel for
			for (int idx = 0; idx < mPointClouds[f]->size(); idx++)
			{
				Eigen::Vector3d m;
				auto &pt = (*mPointClouds[f])[idx];
				Eigen::Vector3d x = { pt.x, pt.y, pt.z };
				do
				{
					double sumg = 0.0;
					Eigen::Vector3d sumxg = Eigen::Vector3d::Zero();
					for (const auto &pt2 : (*mPointClouds[f]))
					{
						Eigen::Vector3d xi = { pt2.x, pt2.y, pt2.z };
						double gval = g((x-xi).squaredNorm() / bandwidth2);
						sumg += gval;
						sumxg += gval*xi;
					}
					m = sumxg / sumg - x;
					x = x + m;
				} while (m.squaredNorm() > 0.000001);
				(*peakCloud)[idx].x = x(0);
				(*peakCloud)[idx].y = x(1);
				(*peakCloud)[idx].z = x(2);
			}
			// populate labels
			pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
			ec.setClusterTolerance(bandwidth);
			ec.setMinClusterSize(mPointClouds[f]->size() / maxNumLabels);
			ec.setMaxClusterSize(mPointClouds[f]->size());
			ec.setInputCloud(peakCloud);
			ec.extract(cluster_indices[f]);
			for (int label = 0; label < cluster_indices[f].size(); label++)
			{
				for (auto idx : cluster_indices[f][label].indices)
				{
					(*labeledPointClouds[f])[idx].label = label;
				}
			}
			// at this point some points may have no label, so take the nearest.
			int numUnassigned = 0;
			for (auto &point : (*labeledPointClouds[f]))
			{
				if (point.label >= cluster_indices[f].size())
				{
					numUnassigned++;
				}
			}
			pcl::KdTreeFLANN<pcl::PointXYZRGBL> kdtree;
			kdtree.setInputCloud(labeledPointClouds[f]);
			for (auto &point : (*labeledPointClouds[f]))
			{
				if (point.label >= cluster_indices[f].size())
				{
					numUnassigned++;
				}
			}
			for (auto &point : (*labeledPointClouds[f]))
			{
				if (point.label >= cluster_indices[f].size())
				{
					std::vector<int> pointIdxNKNSearch(numUnassigned + 1);
					std::vector<float> pointNKNSquaredDistance(numUnassigned + 1);
					const auto numFoundPoints = kdtree.nearestKSearch(point, numUnassigned+1, pointIdxNKNSearch, pointNKNSquaredDistance);
					for (int i = 0; i < numFoundPoints; i++)
					{
						auto &otherPoint = (*labeledPointClouds[f])[pointIdxNKNSearch[i]];
						if (otherPoint.label < cluster_indices[f].size())
						{
							point.label = otherPoint.label;
							break;
						}
					}
				}
			}
		}

		// ----------- rigid registration -------------
		{
			int totalNumPoints = 0;
			std::vector<int> cloudOffsets(mPointClouds.size());
			for(int i=0; i<mPointClouds.size(); i++)
			{
				cloudOffsets[i] = totalNumPoints;
				totalNumPoints += mPointClouds[i]->size();
			}

			constexpr int maxIterations = 1;
			constexpr double errorBound = 0.01;

			std::vector<pcl::KdTreeFLANN<pcl::PointXYZRGBL >> kdtrees(mPointClouds.size());

			LOG_SCOPE_F(INFO, "Starting rigid registration with max %i iterations and an error bound of %f.", maxIterations, errorBound);
			LOG_F(INFO, "%-5s | %-15s | %-15s | %-15s | %-15s | %-15s", "iter", "error", "kdtree time", "var time", "eq build", "eq solve");

			int iteration = 0;
			double error = 1.;

			do
			{
				// update closest points
				auto startTimeKDTree = std::chrono::high_resolution_clock::now();
				for (int f = 0; f < kdtrees.size(); f++)
				{
					kdtrees[f].setInputCloud(labeledPointClouds[f]);
				}
				auto endTimeKDTree = std::chrono::high_resolution_clock::now();
				auto KDTreeRebuildTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTimeKDTree - startTimeKDTree).count();

				// update variances
				auto startTimeVariance = std::chrono::high_resolution_clock::now();
				Eigen::MatrixXd variances(mPointClouds.size(),mPointClouds.size());
				#pragma omp parallel for
				for (int f = 0; f < mPointClouds.size(); f++)
				{
					for (int g = 0; g < mPointClouds.size(); g++)
					{
						variances(f, g) = calculateVariance<pcl::PointXYZRGBL>(labeledPointClouds[f], kdtrees[g]);
					}
				}
				auto endTimeVariance = std::chrono::high_resolution_clock::now();
				auto VarianceRebuildTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTimeVariance - startTimeVariance).count();

				// build equation system
				///@todo optimize via parallel reduction on gpu over each point (xyz)
				auto startTimeEQBuild = std::chrono::high_resolution_clock::now();
				// rotations and translations = 6 paramters per frame
				Eigen::MatrixXd A_ = Eigen::MatrixXd::Zero(6*totalNumPoints, 6*mPointClouds.size());
				Eigen::VectorXd b_ = Eigen::VectorXd::Zero(6*totalNumPoints);
				for(int f = 0; f < labeledPointClouds.size(); f++)
				{
					#pragma omp parallel for
					for(int g = 0; g < labeledPointClouds.size(); g++)
					{
						if(f!=g)
						{
							Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6 * totalNumPoints, 6 * mPointClouds.size());
							Eigen::VectorXd b = Eigen::VectorXd::Zero(6 * totalNumPoints);
							std::vector<int> pointIdxNKNSearch(Nnear);
							std::vector<float> pointNKNSquaredDistance(Nnear);
							
							for(int n = 0; n < labeledPointClouds[f]->size(); n++)
							{
								const auto& yfn = (*labeledPointClouds[f])[n];
								auto numFoundPoints = kdtrees[g].nearestKSearch(yfn, Nnear, pointIdxNKNSearch, pointNKNSquaredDistance);

								for(int i = 0; i < numFoundPoints; i++)
								{
									int m = pointIdxNKNSearch[i];

									
									const auto& ygm = (*labeledPointClouds[g])[m];

									// No reason to precalculate "POld", as it get only calculated once with this implementation
									double pOverVar = calculatePOld(yfn, ygm, kdtrees[g], variances(f,g))/variances(f,g);
									int rowf = 6*(cloudOffsets[f]+n);
									int rowg = 6*(cloudOffsets[g]+m);
									int colf = 6*f;
									int colg = 6*g;

									// fill A
									///@todo use https://eigen.tuxfamily.org/dox/group__TutorialBlockOperations.html
									// I3
									A(rowf + 0, colf + 0) += 1 * pOverVar;
									A(rowf + 1, colf + 1) += 1 * pOverVar;
									A(rowf + 2, colf + 2) += 1 * pOverVar;
									// -y_f,n hat
									A(rowf + 0, colf + 1 + 3) += -1 * pOverVar*-yfn.z; A(rowf + 0, colf + 2 + 3) += -1 * pOverVar*yfn.y;
									A(rowf + 1, colf + 0 + 3) += -1 * pOverVar*yfn.z;  A(rowf + 1, colf + 2 + 3) += -1 * pOverVar*-yfn.x;
									A(rowf + 2, colf + 0 + 3) += -1 * pOverVar*-yfn.y; A(rowf + 2, colf + 1 + 3) += -1 * pOverVar*yfn.x;
									// -I3
									A(rowf + 0, colg + 0 + 3) += -A(rowf + 0, colg + 0);
									A(rowf + 1, colg + 1 + 3) += -A(rowf + 1, colg + 1);
									A(rowf + 2, colg + 2 + 3) += -A(rowf + 2, colg + 2);
									// y_g,m hat
									A(rowf + 0, colg + 1 + 3) += pOverVar*-ygm.z; A(rowf + 0, colg + 2 + 3) += pOverVar*ygm.y;
									A(rowf + 1, colg + 0 + 3) += pOverVar*ygm.z;  A(rowf + 1, colg + 2 + 3) += pOverVar*-ygm.x;
									A(rowf + 2, colg + 0 + 3) += pOverVar*-ygm.y; A(rowf + 2, colg + 1 + 3) += pOverVar*ygm.x;
									// next row
									// y_f,n hat
									A(rowg + 0, colf + 1) += pOverVar*-yfn.z; A(rowg + 0, colf + 2) += pOverVar*yfn.y;
									A(rowg + 1, colf + 0) += pOverVar*yfn.z; A(rowg + 1, colf + 2) += pOverVar*-yfn.x;
									A(rowg + 2, colf + 0) += pOverVar*-yfn.y; A(rowg + 2, colf + 1) += pOverVar*yfn.x;
									// -y_f,n hat * y_f,n hat
									A(rowg + 0, colf + 0 + 3) += -1 * pOverVar*-(yfn.z*yfn.z + yfn.y*yfn.y); A(rowg + 0, colf + 1 + 3) += -1 * pOverVar*yfn.x*yfn.y; A(rowg + 0, colf + 2 + 3) += -1 * pOverVar*yfn.x*yfn.z;
									A(rowg + 1, colf + 0 + 3) += -1 * pOverVar*yfn.x*yfn.y; A(rowg + 1, colf + 1 + 3) += -1 * pOverVar*-(yfn.x*yfn.x + yfn.z*yfn.z); A(rowg + 1, colf + 2 + 3) += -1 * pOverVar*yfn.y*yfn.z;
									A(rowg + 2, colf + 0 + 3) += -1 * pOverVar*yfn.x*yfn.z; A(rowg + 2, colf + 1 + 3) += -1 * pOverVar*yfn.y*yfn.z; A(rowg + 2, colf + 2 + 3) += -1 * pOverVar*-(yfn.x*yfn.x + yfn.y*yfn.y);
									// -y_f,n hat
									A(rowg + 0, colg + 1 + 0) += -A(rowg + 0, colf + 1 + 0); A(rowg + 0, colg + 2 + 0) += -A(rowg + 0, colf + 2 + 0);
									A(rowg + 1, colg + 0 + 0) += -A(rowg + 1, colf + 0 + 0); A(rowg + 1, colg + 2 + 0) += -A(rowg + 1, colf + 2 + 0);
									A(rowg + 2, colg + 0 + 0) += -A(rowg + 2, colf + 0 + 0); A(rowg + 2, colg + 1 + 0) += -A(rowg + 2, colf + 1 + 0);
									// y_f,n hat * y_g,m hat
									A(rowg + 0, colg + 0 + 3) += pOverVar*-(ygm.z*yfn.z + ygm.y*yfn.y); A(rowg + 0, colg + 1 + 3) += pOverVar*ygm.x*yfn.y; A(rowg + 0, colg + 2 + 3) += pOverVar*ygm.x*yfn.z;
									A(rowg + 1, colg + 0 + 3) += pOverVar*yfn.x*ygm.y; A(rowg + 1, colg + 1 + 3) += pOverVar*-(ygm.x*yfn.x + ygm.z*yfn.z); A(rowg + 1, colg + 2 + 3) += pOverVar*ygm.y*yfn.z;
									A(rowg + 2, colg + 0 + 3) += pOverVar*yfn.x*ygm.z; A(rowg + 2, colg + 1 + 3) += pOverVar*yfn.y*ygm.z; A(rowg + 2, colg + 2 + 3) += pOverVar*-(ygm.x*yfn.x + ygm.y*yfn.y);

									// fill b
									b(rowf + 0) += pOverVar*(ygm.x - yfn.x);
									b(rowf + 1) += pOverVar*(ygm.y - yfn.y);
									b(rowf + 2) += pOverVar*(ygm.z - yfn.z);

									b(rowg + 0) += pOverVar*(yfn.y*ygm.z - yfn.z*ygm.y);
									b(rowg + 1) += pOverVar*(yfn.z*ygm.x - yfn.x*ygm.z);
									b(rowg + 2) += pOverVar*(yfn.x*ygm.y - yfn.y*ygm.x);
								}
							}
							#pragma omp critical
							{
								A_ += A;
								b_ += b;
							}
						}
					}
				}
				auto endTimeEQBuild = std::chrono::high_resolution_clock::now();
				auto EQBuildTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTimeEQBuild - startTimeEQBuild).count();

				// solve equation system Ax=b to find rotation and translation deltas
				auto startTimeEQSolve = std::chrono::high_resolution_clock::now();
				Eigen::VectorXd delta = A_.householderQr().solve(b_);
				auto endTimeEQSolve = std::chrono::high_resolution_clock::now();
				auto EQSolveTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTimeEQSolve - startTimeEQSolve).count();

				for (int f = 0;f < labeledPointClouds.size();f++)
				{
					int transOffset = f * 6;
					int rotOffset = f * 6 + 3;

					Eigen::Vector3d l(delta(rotOffset), delta(rotOffset + 1), delta(rotOffset + 2));
					Eigen::Vector3d m(delta(transOffset), delta(transOffset + 1), delta(transOffset + 2));
					double phi = l.norm();
					Eigen::Affine3d trafo = Eigen::Affine3d::Identity();
					if (phi > DBL_EPSILON || phi < -DBL_EPSILON)
					{
						double phi2 = phi*phi;
						Eigen::Matrix3d lhat;
						lhat << 0, -l(2), l(1),
							l(2), 0, -l(0),
							-l(1), l(0), 0;
						auto lhat2 = lhat*lhat;
						auto R = Eigen::Matrix3d::Identity() + lhat*sin(phi) / phi + lhat2*(1 - cos(phi)) / phi2;
						trafo(0, 0) = R(0, 0);trafo(0, 1) = R(0, 1);trafo(0, 2) = R(0, 2);
						trafo(1, 0) = R(1, 0);trafo(1, 1) = R(1, 1);trafo(1, 2) = R(1, 2);
						trafo(2, 0) = R(2, 0);trafo(2, 1) = R(2, 1);trafo(2, 2) = R(2, 2);

						auto t = ((Eigen::Matrix3d::Identity() - R)*lhat*m + l*l.transpose()*m) / phi2;
						trafo(0, 3) = t(0);
						trafo(1, 3) = t(1);
						trafo(2, 3) = t(2);
					}
					else
					{
						trafo(0, 3) = m(0);
						trafo(1, 3) = m(1);
						trafo(2, 3) = m(2);
					}
					pcl::transformPointCloud(*labeledPointClouds[f], *labeledPointClouds[f], trafo);
				}

				error = delta.squaredNorm();
				iteration++;
				LOG_F(INFO, "%-5i | %-15f | %-15i | %-15i | %-15i | %-15i", iteration, error, KDTreeRebuildTime, VarianceRebuildTime, EQBuildTime);
			} while (iteration < maxIterations && error > errorBound);
		}

		// non-rigid registration
		nonrigidtest(labeledPointClouds, cluster_indices);
		/*
		{
			constexpr int maxIterations = 1;
			constexpr double errorBound = 0.05;

			std::vector<pcl::KdTreeFLANN<pcl::PointXYZRGBL >> kdtrees(mPointClouds.size());

			LOG_SCOPE_F(INFO, "Starting rigid registration with max %i iterations and an error bound of %f.", maxIterations, errorBound);
			
			do
			{
				// update kd tree
				auto startTimeKDTree = std::chrono::high_resolution_clock::now();
				for (int i = 0;i < kdtrees.size(); i++)
				{
					kdtrees[i].setInputCloud(labeledPointClouds[i]);
				}
				auto endTimeKDTree = std::chrono::high_resolution_clock::now();
				auto KDTreeRebuildTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTimeKDTree - startTimeKDTree).count();

				// http://www.andres.sc/publications/opengm-2.0.2-beta-manual.pdf
				opengm::SimpleDiscreteSpace<> space();
				// precompute components of Q for all y and f
				// build graph
				// execute alpha expansion
			} while (0);
		}
		*/

		// merge point clouds
		pcl::PointCloud<pcl::PointXYZRGBL> completePointCloud;
		for (int f = 0; f < labeledPointClouds.size(); f++)
		{
			LOG_S(INFO) << "Merging " << f + 1 << "/" << mPointClouds.size() << std::endl;
			completePointCloud += *labeledPointClouds[f];
			pcl::copyPointCloud(*labeledPointClouds[f], *mPointClouds[f]);
			std::string filename = "data/point_cloud_rigid_registered";
			LOG_S(INFO) << "Saving back results...";
			try {

				pcl::io::savePLYFile(filename + ".ply", *labeledPointClouds[f]);
				pcl::io::savePCDFile(filename + ".pcd", *labeledPointClouds[f]);
				LOG_S(INFO) << "Successfully saved filtered point cloud to " << filename;
			}
			catch (std::runtime_error& ex) {
				LOG_S(INFO) << "Exception converting image to point cloud format: " << ex.what();
				return;
			}
		}

		std::string filename = "data/point_cloud_merged";
		LOG_S(INFO) << "Saving back results...";
		try {

			pcl::io::savePLYFile(filename + ".ply", completePointCloud);
			pcl::io::savePCDFile(filename + ".pcd", completePointCloud);
			LOG_S(INFO) << "Successfully saved filtered point cloud to " << filename;
		}
		catch (std::runtime_error& ex) {
			LOG_S(INFO) << "Exception converting image to point cloud format: " << ex.what();
			return;
		}
	}


	void nonrigidtest(std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBL>>> labeledPointClouds, std::vector<std::vector<pcl::PointIndices>> &cluster_indices)
	{
		std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBL>>> labeledPointCloudsTransformed(labeledPointClouds.size()); //helper cloud
		//typedef std::tuple<pcl::PointXYZ, uint32_t, uint32_t, uint32_t> JointMeta; // position, label, other label, strength
		typedef pcl::CentroidPoint<pcl::PointXYZ> CentroidXYZ;
		std::vector< std::map<uint32_t, std::map<uint32_t, CentroidXYZ> > > jointEstimations(labeledPointClouds.size());

		std::vector<int> pointIdxNKNSearch(5);
		std::vector<float> pointNKNSquaredDistance(5);

		for (int f = 0;f < labeledPointClouds.size();f++)
		{
			auto &currentCloud = *(labeledPointClouds[f]);

			// construct helper cloud
			labeledPointCloudsTransformed[f] = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBL>>();
			pcl::copyPointCloud(currentCloud, *(labeledPointCloudsTransformed[f]));

			// approximate joint locations
			pcl::KdTreeFLANN<pcl::PointXYZRGBL> kdtree;
			kdtree.setInputCloud(labeledPointClouds[f]);
			//search for "8-neighborhood" as we reconstructed these clouds from depth images...
			for (int label = 0;label < cluster_indices[f].size(); label++)
			{
				for (const auto &index : cluster_indices[f][label].indices)
				{
					const auto point = currentCloud[index];
					const auto numFoundPoints = kdtree.nearestKSearch(point, 5, pointIdxNKNSearch, pointNKNSquaredDistance);
					for (int i = 0; i < numFoundPoints; i++ )
					{
						if (pointNKNSquaredDistance[i] == 0.0) continue;

						const auto &otherIndex = pointIdxNKNSearch[i];
						const auto &otherPoint = currentCloud[otherIndex];
						const auto &otherLabel = otherPoint.label;
						// we found an edge point
						if (label != otherLabel)
						{
							auto smallerLabel = std::min<uint32_t>(label, otherPoint.label);
							auto greaterLabel = std::max<uint32_t>(label, otherPoint.label);
							jointEstimations[f][smallerLabel][greaterLabel].add(pcl::PointXYZ(point.x, point.y, point.z));
							jointEstimations[f][smallerLabel][greaterLabel].add(pcl::PointXYZ(otherPoint.x, otherPoint.y, otherPoint.z));
						}
					}
				}
			}
		}

		constexpr int maxIterations = 1;
		constexpr double errorBound = 0.01;

		std::vector<pcl::KdTreeFLANN<pcl::PointXYZRGBL>> kdtrees(labeledPointClouds.size());
		//std::vector<int> pointIdxNKNSearch(Nnear);
		//std::vector<float> pointNKNSquaredDistance(Nnear);
		LOG_SCOPE_F(INFO, "Starting non-rigid registration with max %i iterations and an error bound of %f.", maxIterations, errorBound);

		do
		{
			// update kd tree
			auto startTimeKDTree = std::chrono::high_resolution_clock::now();
			#pragma omp parallel for
			for (int i = 0;i < kdtrees.size(); i++)
			{
				kdtrees[i].setInputCloud(labeledPointClouds[i]);
			}
			auto endTimeKDTree = std::chrono::high_resolution_clock::now();
			auto KDTreeRebuildTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTimeKDTree - startTimeKDTree).count();

			// update variance
			auto startTimeVariance = std::chrono::high_resolution_clock::now();
			Eigen::VectorXd variances(labeledPointClouds.size());
			#pragma omp parallel for
			for (int f = 0; f < labeledPointClouds.size(); f++)
			{
				variances(f) = calculateVariance<pcl::PointXYZRGBL>(labeledPointClouds[f], kdtrees[f]);
			}
			auto endTimeVariance = std::chrono::high_resolution_clock::now();
			auto VarianceRebuildTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTimeVariance - startTimeVariance).count();
			
			// precompute components of Q for all y and f
			// build graph (http://www.andres.sc/publications/opengm-2.0.2-beta-manual.pdf)
			for (int f = 0; f < labeledPointClouds.size(); f++)
			{
				/*
				// def
				auto &currentCloud = *labeledPointClouds[f];
				typedef opengm::SimpleDiscreteSpace<> Space;
				typedef opengm::meta::TypeListGenerator<opengm::ExplicitFunction<double>, opengm::PottsFunction<double>>::type FunctionTypeList;
				typedef opengm::GraphicalModel<double, opengm::Adder, FunctionTypeList, Space> Model;
				typedef opengm::MinSTCutBoost < size_t, double, opengm::PUSH_RELABEL > MinStCutType;
				typedef opengm::GraphCut< Model, opengm::Minimizer, MinStCutType >	MinGraphCut;
				typedef opengm::AlphaExpansion < Model, MinGraphCut > MinAlphaExpansion;

				constexpr int numLabels = 20;

				// build model
				Space space(currentCloud.size(), numLabels);
				Model gm(space);

				// data term
				for (int n = 0; n < currentCloud.size(); n++)
				{
					std::array<size_t, 1> shape = { numLabels };
					opengm::ExplicitFunction<double> d(shape.begin(), shape.end());
					//auto numFoundPoints = kdtrees[f].nearestKSearch(currentCloud[n], Nnear, pointIdxNKNSearch, pointNKNSquaredDistance);
					double distSum = 0.0;
					for (int m = 0; m < currentCloud.size(); m++)
					{
						Eigen::Vector3d yfn = { currentCloud[n].x ,currentCloud[n].y,currentCloud[n].z };
						Eigen::Vector3d yfm = { currentCloud[m].x ,currentCloud[m].y,currentCloud[m].z };
						distSum += exp((yfn-yfm).squaredNorm() / (-2 * variances(f)));
						distSum += (yfn - yfm).squaredNorm();
					}
					std::array<size_t, 1> index = { n };
					for (int l = 0;l < numLabels;l++)
					{
						d(l) = -log(distSum);
					}
					Model::FunctionIdentifier fidD = gm.addFunction(d);
					gm.addFactor(fidD, index.begin(), index.end());
				}

				// regularization term
				opengm::PottsFunction<double> P(numLabels, numLabels, 0.0, 1.0);
				Model::FunctionIdentifier fidP = gm.addFunction(P);
				for (int n = 0; n < currentCloud.size(); n++)
				{
					auto numFoundPoints = kdtrees[f].nearestKSearch(currentCloud[n], Nnear, pointIdxNKNSearch, pointNKNSquaredDistance);
					for (int i = 0; i < numFoundPoints; i++)
					{
						auto m = pointIdxNKNSearch[i];
						if (m != n)
						{
							std::array<size_t, 2> indices = { n, m };
							std::sort(indices.begin(), indices.end());
							gm.addFactor(fidP, indices.begin(), indices.end());
						}
					}
				}

				// execute alpha expansion
				MinAlphaExpansion ae(gm);
				//ae.setStartingPoint();
				ae.infer();
				std::vector<size_t> labels(currentCloud.size());
				ae.arg(labels);

				for (int n = 0;n < labels.size();n++)
				{
					currentCloud[n].label = labels[n];
				}
				*/
				boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
				viewer->setBackgroundColor(0, 0, 0);
				viewer->addCoordinateSystem(1.0);

				// show current cloud
				{
					pcl::visualization::PointCloudColorHandlerLabelField<pcl::PointXYZRGBL> hlabel(labeledPointClouds[f]);
					viewer->addPointCloud<pcl::PointXYZRGBL>(labeledPointClouds[f], hlabel, "sample cloud" + std::to_string(f));
					viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud" + std::to_string(f));
					int runningIndex = 0;
					for (auto &pt : *(labeledPointClouds[f]))
					{
						if (pt.label >= cluster_indices[f].size())
						{
							pcl::PointXYZ centroid = { pt.x, pt.y, pt.z };
							viewer->addSphere(centroid, 0.015, "sphere-" + std::to_string(f) + "-" + std::to_string(runningIndex));
							runningIndex++;
						}

					}
					for (auto const &ent1 : jointEstimations[f]) 
					{
						for (auto const &ent2 : ent1.second) 
						{
							pcl::PointXYZ centroid;
							ent2.second.get(centroid);
							viewer->addSphere(centroid, 0.025, "sphere-" + std::to_string(f) + "-" + std::to_string(ent1.first) + "-" + std::to_string(ent2.first));
						}
					}
				}
				viewer->initCameraParameters();

				while (!viewer->wasStopped())
				{
					viewer->spinOnce(100);
					boost::this_thread::sleep(boost::posix_time::microseconds(100000));
				}
			}
		} while (0);
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


	void loadFilteredPointCloudsFromDisk(fs::path dataFolder)
	{
		for (auto& p : fs::directory_iterator(dataFolder))
		{
			if (p.path().has_filename())
			{
				auto filename = p.path().filename().generic_string();
				auto indexPosEnd = filename.find("_filtered.pcd");
				if (filename.find("point_cloud_") == 0 && indexPosEnd != -1)
				{
					auto index = stoi(filename.substr(12, indexPosEnd - 12));
					LOG_S(INFO) << "Loading preprocessed point cloud " << index << " from file " << p;
					if (pcl::io::loadPCDFile(p.path().generic_string(), *mPointClouds[index - 1]) != -1)
					{
						LOG_S(INFO) << "Loading successful!";
					}
					//else // error to cout by pcl
				}
			}
		}
	}

	void loadLowResCloudsFromDisk(fs::path dataFolder)
	{
		for (auto& p : fs::directory_iterator(dataFolder))
		{
			if (p.path().has_filename())
			{
				auto filename = p.path().filename().generic_string();
				auto indexPosEnd = filename.find("_downsampled.pcd");
				if (filename.find("point_cloud_") == 0 && indexPosEnd != -1)
				{
					auto index = stoi(filename.substr(12, indexPosEnd - 12));
					LOG_S(INFO) << "Loading preprocessed point cloud " << index << " from file " << p;
					if (pcl::io::loadPCDFile(p.path().generic_string(), *mPointClouds[index - 1]) != -1)
					{
						LOG_S(INFO) << "Loading successful!";
					}
					//else // error to cout by pcl
				}
			}
		}
	}
};



#include <algorithm>
#include <numeric>
int _main_(int _argc, char** _argv)
{
	const char* const appTitle = "Human Body Reconstruction";

	Args args(_argc, _argv);

	//setup logging
	loguru::init(_argc, _argv);
	loguru::add_file("everything.log", loguru::Append, loguru::Verbosity_MAX);

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
		cv::ocl::setUseOpenCL(true);
		LOG_S(INFO) << "OpenCV " << CV_VERSION << " OpenCL active: " << cv::ocl::haveOpenCL();
		cv::ocl::Context context;
		if (!context.create(cv::ocl::Device::TYPE_GPU))
		{
			LOG_S(INFO) << "Failed creating the context...";
			//return;
		}

		LOG_S(INFO) << context.ndevices() << " GPU devices are detected." << endl; //This bit provides an overview of the OpenCL devices you have in your computer
		for (int i = 0; i < context.ndevices(); i++)
		{
			cv::ocl::Device device = context.device(i);
			LOG_S(INFO) << "name:              " << device.name();
			LOG_S(INFO) << "available:         " << device.available();
			LOG_S(INFO) << "imageSupport:      " << device.imageSupport();
			LOG_S(INFO) << "OpenCL_C_Version:  " << device.OpenCL_C_Version();
			LOG_S(INFO) << endl;
		}

		cv::ocl::Device(context.device(0)); //Here is where you change which GPU to use (e.g. 0 or 1)
		KinectDataProvider KDP;

		bool runningCapture = false;
		ReconstructionState reconstruction(7, 6);

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


					if (runningCapture)
					{
						if (reconstruction.targetAngle() - 2. < bodyAngle && bodyAngle < reconstruction.targetAngle() + 2.)
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
								reconstruction.constructPointClouds(&KDP);
								doOnce = false;
							}
						}
					}
				}
			}
			catch(...)
			{
				//std::cout << "Interaction with the kinect failed." << std::endl;
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
					ImGui::Checkbox("Show Triplanar View", &showTriplanarView);
					ImGui::Separator();
				ImGui::End();

				ImGui::Begin("Point Cloud Panel");
					ImGui::BeginGroup();
						if (ImGui::Button("Preprocess point clouds"))
						{
							reconstruction.preprocessPointClouds();
						}
					ImGui::EndGroup();
					ImGui::BeginGroup();
						if (ImGui::Button("Downsample point clouds"))
						{
							reconstruction.downsamplePointClouds();
						}
					ImGui::EndGroup();
					ImGui::BeginGroup();
						if (ImGui::Button("View point clouds"))
						{
							reconstruction.showPointClouds();
						}
					ImGui::EndGroup();
				ImGui::End();

				ImGui::Begin("Reconstruction Panel");
					ImGui::BeginGroup();
						ImGui::Value("Current angle", bodyAngle);

						ImGui::SameLine();
						ImGui::Value("Target angle", reconstruction.targetAngle());
					ImGui::EndGroup();
					ImGui::ProgressBar(0.);
					ImGui::BeginGroup();
						if (ImGui::Button("Start capture"))
						{
							runningCapture = true;
						}
						ImGui::SameLine();
						//if (ImGui::Button("Record chunk"))
						{

						}
						ImGui::SameLine();
						if (ImGui::Button("Cancel capture"))
						{
							runningCapture = false;
							reconstruction.reset();
						}
						if (ImGui::Button("Load captured data"))
						{
							reconstruction.loadFilteredPointCloudsFromDisk("data");
						}
						ImGui::SameLine();
						if (ImGui::Button("Load test data"))
						{
							reconstruction.loadLowResCloudsFromDisk("data");
						}
						if (ImGui::Button("Register point clouds"))
						{
							reconstruction.reconstructAvatar();
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
						//ImGui::Checkbox("Show Skeletons", &showSkeletons);
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
