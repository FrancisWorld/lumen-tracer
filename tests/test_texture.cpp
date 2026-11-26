#include "lumen/texture.hpp"
#include "testing.hpp"

using namespace lumen;

TEST_CASE(texture, srgb_transfer_functions_invert_each_other) {
    for (int i = 0; i <= 20; ++i) {
        const Real v = static_cast<Real>(i) / 20;
        CHECK_NEAR(linear_to_srgb(srgb_to_linear(v)), v, 1e-4);
    }
    CHECK_NEAR(srgb_to_linear(0.5F), 0.2140, 1e-3);  // well-known mid-grey value
    CHECK_NEAR(linear_to_srgb(2.0F), 1.0, 1e-6);        // clamps out-of-range input
    CHECK_NEAR(linear_to_srgb(-1.0F), 0.0, 1e-6);
}

TEST_CASE(texture, checker_alternates_cells) {
    const Texture t = CheckerTexture{Vec3(1), Vec3(0), 2};  // 2 cell pairs per unit -> 4 cells per unit
    CHECK_NEAR(evaluate(t, {0.05F, 0.05F}).x, 1.0, 0);
    CHECK_NEAR(evaluate(t, {0.30F, 0.05F}).x, 0.0, 0);
    CHECK_NEAR(evaluate(t, {0.30F, 0.30F}).x, 1.0, 0);
    CHECK_NEAR(evaluate(t, {-0.05F, 0.05F}).x, 0.0, 0);  // negative uv stays well defined
}

TEST_CASE(texture, constant_ignores_uv) {
    const Texture t = ConstantTexture{{0.2F, 0.4F, 0.6F}};
    CHECK_NEAR(evaluate(t, {123.0F, -4.0F}).y, 0.4, 1e-6);
}

TEST_CASE(texture, image_is_bilinear_wrapping_and_v_up) {
    // 2x2 raw (non-sRGB) gray image: top row 0, 255; bottom row 255, 0.
    Image8 img{2, 2, 1, {0, 255, 255, 0}};
    const Texture t = ImageTexture::from_image(img, false, {1, 1});
    // Texel centres: v near 1 is the top row.
    CHECK_NEAR(evaluate(t, {0.25F, 0.75F}).x, 0.0, 1e-5);
    CHECK_NEAR(evaluate(t, {0.75F, 0.75F}).x, 1.0, 1e-5);
    CHECK_NEAR(evaluate(t, {0.25F, 0.25F}).x, 1.0, 1e-5);
    // Halfway between two texel centres blends.
    CHECK_NEAR(evaluate(t, {0.5F, 0.75F}).x, 0.5, 1e-5);
    // Repeat: uv + 1 gives the same colour.
    CHECK_NEAR(evaluate(t, {1.25F, 1.75F}).x, evaluate(t, {0.25F, 0.75F}).x, 1e-5);
    CHECK_NEAR(evaluate(t, {-0.75F, 0.75F}).x, evaluate(t, {0.25F, 0.75F}).x, 1e-5);
}

TEST_CASE(texture, image_conversion_handles_channels_and_srgb) {
    Image8 rgba{1, 1, 4, {255, 128, 0, 77}};
    const auto t = ImageTexture::from_image(rgba, true, {1, 1});
    CHECK_NEAR(t.texels[0].x, 1.0, 1e-5);
    CHECK_NEAR(t.texels[0].y, srgb_to_linear(128.0F / 255.0F), 1e-5);
    CHECK_NEAR(t.texels[0].z, 0.0, 1e-5);
    Image8 gray_alpha{1, 1, 2, {255, 9}};
    const auto g = ImageTexture::from_image(gray_alpha, true, {1, 1});
    CHECK_NEAR(g.texels[0].x, 1.0, 1e-5);
    CHECK_NEAR(g.texels[0].z, 1.0, 1e-5);
}
