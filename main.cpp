#include <iostream>

#include <algorithm>

#include <array>

#include  <type_traits>

#include "image.h"

#include <boost/range/combine.hpp>


Image< unsigned char > average_filter(const Image< unsigned char > &input)
{
    Image< unsigned char > output(input.width, input.height);

    for (auto tpl : boost::combine(blocks(output, 1), blocks(input, 1)))
    {
        const auto &block = boost::get<1>(tpl);
        boost::get<0>(tpl)(0,0) = ( block(-1, -1) + block(0, -1) + block(+1, -1)
                                  + block(-1,  0) + block(0,  0) + block(+1,  0)
                                  + block(-1, +1) + block(0, +1) + block(+1, +1) )/9;
    }

    return output;
}

Image< unsigned char > median_filter(const Image< unsigned char > &input)
{
    Image< unsigned char > output(input.width, input.height);

    for (auto tpl : boost::combine(blocks(output, 1), blocks(input, 1)))
    {
        const auto &block = boost::get<1>(tpl);

        std::array<unsigned char, 9> pixels = {
                block(-1, -1), block(0, -1), block(+1, -1),
                block(-1,  0), block(0,  0), block(+1,  0),
                block(-1, +1), block(0, +1), block(+1, +1)
        };

        std::sort(begin(pixels), end(pixels));
        boost::get<0>(tpl)(0,0) = pixels[4];

    }
    return output;
}



Image< unsigned char > convolute(const Image< unsigned char > &input, const std::array<std::ptrdiff_t, 9> &filter)
{
    Image< unsigned char > output(input.width, input.height);

    for (auto tpl : boost::combine(blocks(output, 1), blocks(input, 1)))
    {
        const auto &block = boost::get<1>(tpl);
        boost::get<0>(tpl)(0,0) = ( block(-1, -1) * filter[0] + block(0, -1) * filter[1] + block(+1, -1) * filter[2]
                                  + block(-1,  0) * filter[3] + block(0,  0) * filter[4] + block(+1,  0) * filter[5]
                                  + block(-1, +1) * filter[6] + block(0, +1) * filter[7] + block(+1, +1) * filter[8] );
    }

    return output;
}


template<typename T>
std::array<T,2> rotate(const std::array<T,2> &point, const std::array<T,2> &center, T theta)
{
    const T cos_theta = cos(theta);
    const T sin_theta = sin(theta);
    const auto dx = point[0] - center[0];
    const auto dy = point[1] - center[1];

    return { dx * cos_theta - dy * sin_theta + center[0],
             dx * sin_theta + dy * cos_theta + center[1] };
}

void rotate(const Image< unsigned char > &input, Image< unsigned char > &output, float theta)
{
    const std::array<float,2> center = { input.width/2.0f, input.height/2.0f };
    for (auto px : pixels(output))
    {
        const float x = px.x();
        const float y = px.y();
        auto p = rotate( {x, y}, center, theta );
        if (p[0] >= 0 && p[0] < output.width && p[1] >= 0 && p[1] < output.height)
        {
            output(x, y) = output(p[0], p[1]);
        }
    }
}

std::vector<std::size_t> histogram(const Image< unsigned char > &input)
{
    const auto values_count = 1 << Image< unsigned char >::depth;
    std::vector<std::size_t> hist( values_count, 0 );

    for (const auto &px : pixels(input))
    {
        hist[ px ] += 1;
    }

    return hist;
}

Image< unsigned char > histogram_img(const std::vector<std::size_t> &hist)
{
    const auto values_count = 1 << png::pixel_traits< png::gray_pixel >::get_bit_depth();

    const auto max = *std::max_element(begin(hist), end(hist));

    Image< unsigned char > output(hist.size(), max);

    for (auto px : pixels(output))
    {
        if ( hist[px.x()] < max-1-px.y())
        {
            px = 255;
        }
        else
        {
            px = 0;
        }
    }

    return output;
}

Image< unsigned char > equalize_histogram(const Image< unsigned char > &input)
{
    const auto hist = histogram(input);

    const auto num_pixels = input.width *input.height;

    auto cummulative_hist = hist;

    cummulative_hist[0] = hist[0];
    for (size_t i = 1; i < hist.size(); i++)
    {
        cummulative_hist[i] = hist[i] + cummulative_hist[i-1];
    }

    const auto alpha = 255.0f / num_pixels;

    std::transform(begin(cummulative_hist), end(cummulative_hist), begin(cummulative_hist), [&](unsigned char px){
        return 0.5f + px * alpha;
    });

    Image< unsigned char > output(input.width, input.height);

    for (auto px : boost::combine(pixels(output), pixels(input)))
    {
        boost::get<0>(px) = cummulative_hist[ boost::get<1>(px) ];
    }

    return output;
}

int main()
{
    Image< unsigned char > input = read_image("input.png");
    //input.width = 5;
    //input.height = 4;
    //input.data.resize(input.width * input.height);
    {
        auto hist = histogram(input);
        auto hist_img = histogram_img(hist);
        write_image(hist_img, "input_hist.png");
    }

    //auto output = equalize_histogram(input);
    //auto output = median_filter(input);
    //auto output = average_filter(input);
/*
   */

    const auto blured = average_filter(input);

write_image(blured, "blured.png");

     auto output = convolute(blured, {  1, 1,  1,
                                     1,  -8, 1,
                                      1, 1,  1 });

write_image(output, "convoluted.png");

    for (unsigned i=0; i<input.data.size(); ++i)
    {
        output.data[i] = input.data[i] + output.data[i];
    }

    {
        auto hist = histogram(output);
        auto hist_img = histogram_img(hist);
        write_image(hist_img, "output_hist.png");
    }

    write_image(output, "output.png");

    std::cout << "Hello World! 7" << std::endl;
}
