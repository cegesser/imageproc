#pragma once

#include <vector>
#include <type_traits>

#include <boost/iterator/iterator_facade.hpp>
#include <boost/range/iterator_range_core.hpp>
#include <png++/image.hpp>

template<typename Pixel>
struct Image
{
    Image(size_t width, size_t height) : width(width), height(height), data(width*height*sizeof(Pixel))  {}

    size_t width;
    size_t height;
    std::vector<Pixel> data;

    size_t size() const { return data.size(); }

    static const size_t depth = std::numeric_limits< Pixel >::digits;

    Pixel &operator()(ptrdiff_t x, ptrdiff_t y) { return data[ x + y * width ]; }
    Pixel operator()(ptrdiff_t x, ptrdiff_t y) const { return data[ x + y * width ]; }
};

//IO

Image<unsigned char> read_image(const std::string &name)
{
    png::image< png::gray_pixel > input("input.png");

    Image<unsigned char> result(input.get_width(), input.get_height());
    result.data.clear();

    for (size_t y=0; y<input.get_height(); ++y)
    {
        const auto &row = input[y];

        result.data.insert(end(result.data), begin(row), end(row));
    }

    return result;
}

void write_image(const Image<unsigned char> &image, const std::string &name)
{
    png::image< png::gray_pixel > output(image.width, image.height);

    for (size_t y=0; y<output.get_height(); ++y)
    {
        std::copy(begin(image.data) + y*image.width,
                  begin(image.data) + y*image.width + image.width,
                  begin(output[y]));
    }

    output.write(name);
}

//Pixel iteration

struct end_tag {};
struct begin_tag {};

template<typename PixelT>
struct PixelRef
{
    using Pixel = typename std::remove_const<PixelT>::type;

    using ImageType = typename std::conditional< std::is_const<PixelT>::value,  const Image<Pixel>, Image<Pixel>>::type;

  	size_t i;
    ImageType &image;

    inline size_t x() const { return i % image.width; }
  	inline size_t y() const { return i / image.width; }

    inline PixelRef &operator=(Pixel value) { image.data[i] = value; return *this; }
    inline operator PixelT &() { return image.data[i]; }
  	inline operator Pixel () const { return image.data[i]; }
};

template<typename PixelT>
class PixelIterator : public boost::iterator_facade< PixelIterator<PixelT>, PixelRef<PixelT>, boost::random_access_traversal_tag, PixelRef<PixelT> >
{
    using Pixel = typename std::remove_const<PixelT>::type;

    using ImageType = typename std::conditional< std::is_const<PixelT>::value,  const Image<Pixel>, Image<Pixel>>::type;

    ImageType &image;
    size_t i;

public:
    PixelIterator(ImageType &image, const begin_tag &)
        : image(image), i(0) { }

    PixelIterator(ImageType &image, const end_tag &)
        : image(image), i(image.data.size()) { }

    void increment() { ++i; }
    void decrement() { --i; }

    void advance(std::ptrdiff_t n) { i += n; }
    std::ptrdiff_t distance_to(const PixelIterator &other) { return other.i - i; }

    bool equal(PixelIterator const& other) const
    {
        return this->i == other.i;
    }

    PixelRef<PixelT> dereference() const { return { i, image }; }
};


template<typename Pixel>
boost::iterator_range<PixelIterator<Pixel>> pixels( Image<Pixel> &image ) {
    return { PixelIterator<Pixel>(image, begin_tag()), PixelIterator<Pixel>(image, end_tag()) };
}

template<typename Pixel>
boost::iterator_range<PixelIterator<const Pixel>> pixels( const Image<Pixel> &image ) {
    return { PixelIterator<const Pixel>(image, begin_tag()), PixelIterator<const Pixel>(image, end_tag()) };
}

//Block iteration
template<typename PixelT>
struct BlockRef
{
    using Pixel = typename std::remove_const<PixelT>::type;

    using ImageType = typename std::conditional< std::is_const<PixelT>::value,  const Image<Pixel>, Image<Pixel>>::type;

    std::size_t i;
    ImageType &image;

    inline size_t x() const { return i % image.width; }
  	inline size_t y() const { return i / image.width; }

    PixelT &operator()(ptrdiff_t x, ptrdiff_t y) { return image.data[ i + x + y * image.width ]; }
    Pixel operator()(ptrdiff_t x, ptrdiff_t y) const { return image.data[ i + x + y * image.width ]; }
};

template<typename PixelT>
class BlockIterator : public boost::iterator_facade< BlockIterator<PixelT>, BlockRef<PixelT>, boost::forward_traversal_tag, BlockRef<PixelT> >
{
    using Pixel = typename std::remove_const<PixelT>::type;

    using ImageType = typename std::conditional< std::is_const<PixelT>::value,  const Image<Pixel>, Image<Pixel>>::type;

    ImageType &image;
    size_t neighborhood;

    size_t i;
    size_t x;

public:
    BlockIterator(ImageType &image, size_t neighborhood, const begin_tag &)
        : image(image), neighborhood(neighborhood), x(neighborhood)
    {
        const auto y = neighborhood;
        i = x + y*image.width;
    }

    BlockIterator(ImageType &image, size_t neighborhood, const end_tag &)
        : image(image), neighborhood(neighborhood), x(image.width-2*neighborhood)
    {
        const auto y = image.height - neighborhood;
        i = y*image.width + neighborhood;
    }

    void increment() {
        ++x;
        ++i;
        if (x + neighborhood == image.width) {
            x = neighborhood;
            i += neighborhood + neighborhood;
        }
    }

    bool equal(BlockIterator const& other) const
    {
        return this->i == other.i;
    }

    BlockRef<PixelT> dereference() const { return { i, image }; }
};

template<typename Pixel>
boost::iterator_range<BlockIterator<Pixel>> blocks( Image<Pixel> &image, size_t neighborhood ) {
    return { BlockIterator<Pixel>(image, neighborhood, begin_tag()),
             BlockIterator<Pixel>(image, neighborhood, end_tag()) };
}

template<typename Pixel>
boost::iterator_range<BlockIterator<const Pixel>> blocks( const Image<Pixel> &image, size_t neighborhood ) {
        return { BlockIterator<const Pixel>(image, neighborhood, begin_tag()),
                 BlockIterator<const Pixel>(image, neighborhood, end_tag()) };
}
