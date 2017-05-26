#pragma once


//! Helper class to handle bgfx images
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

	void writePixel(const int x, const int y, const uint8_t red, const uint8_t green, const uint8_t blue, const uint8_t alpha)
	{
		auto pos = 4 * (y*width() + x);
		mBuffer->data[pos] = red;
		mBuffer->data[pos + 1] = green;
		mBuffer->data[pos + 2] = blue;
		mBuffer->data[pos + 3] = alpha;
	}

	void writePixel(const int x, const int y, const uint8_t grey)
	{
		writePixel(x, y, grey, grey, grey, 255);
	}

	void writePixel(const int x, const int y, const uint8_t red, const uint8_t green, const uint8_t blue)
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
