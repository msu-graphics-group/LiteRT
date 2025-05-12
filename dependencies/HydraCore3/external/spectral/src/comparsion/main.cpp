#include <spec/spectral_util.h>
#include <spec/basic_spectrum.h>
#include <spec/conversions.h>
#include <spec/metrics.h>
#include <internal/serialization/csv.h>
#include <internal/math/math.h>
#include <internal/common/util.h>
#include <internal/common/format.h>
#include <upsample/glassner_naive.h>
#include <upsample/smits.h>
#include <upsample/sigpoly.h>
#include <upsample/functional/fourier.h>
#include <stdexcept>
#include <iomanip>
#include <filesystem>
#include <memory>
#include <numeric>
#include <iostream>
#include <cstring>
#include <fstream>
#include <chrono>
#ifdef SPECTRAL_ENABLE_OPENMP
#include <omp.h>
#endif

using namespace spec;
namespace fs = std::filesystem;

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;

std::unique_ptr<IUpsampler> get_upsampler_by_name(const std::string &method_name)
{
    IUpsampler *ptr = nullptr;
    if(method_name == "glassner") {
        ptr = new GlassnerUpsampler();
    }
    else if(method_name == "sigpoly") {
        ptr = new SigPolyUpsampler();
    }
    else if(method_name == "smits") {
        ptr = new SmitsUpsampler();
    }
    return std::unique_ptr<IUpsampler>(ptr);
}

void load_spec_ds(const std::string &path, std::vector<Float> &wavelenghts, std::vector<BasicSpectrum> &spectra) {
    std::ifstream file_in{path};

    auto header = *csv::parse_line_m<spec::Float, csv::skip>(file_in);

    auto table = csv::load_as_vector_m<spec::Float, csv::skip>(file_in);
    file_in.close();

    wavelenghts = std::get<0>(header);
    for(const auto &entry : table) {
        BasicSpectrum spectrum;
        const std::vector<spec::Float> &values = std::get<0>(entry);
        for(unsigned i = 0; i < wavelenghts.size(); ++i) {
            spectrum.set(wavelenghts[i], values[i]);
        }
        spectra.push_back(spectrum);
    }
}

void load_vec_ds(const std::string &path, std::vector<vec3> &data) {
    std::ifstream file_in{path};

    auto table = csv::load_as_vector<int, int, int>(file_in, ',', 1);
    file_in.close();

    for(const auto &entry : table) {
        data.push_back({Float(std::get<0>(entry)) / 255.0f, Float(std::get<1>(entry)) / 255.0f, Float(std::get<2>(entry)) / 255.0f});
    }
}

Float stddev(const std::vector<Float> &vals, Float mean) {
    double disp = 0.0;
    for(Float v : vals) {
        disp += (v - mean) * (v - mean);
    }
    return Float(std::sqrt(disp / double(vals.size())));
}

void dataset_reupsample(const IUpsampler &upsampler, const std::string &method_name)
{
    std::vector<BasicSpectrum> in_spectra;
    std::vector<Float> wavelenghts;
    load_spec_ds("input/munsell380_800_1.csv", wavelenghts, in_spectra);
    std::vector<BasicSpectrum> out_spectra(in_spectra.size());

    std::ofstream output_file("output/comparsion_ds_result_" + method_name + ".csv");
    std::ofstream result_spectra_file("output/comparsion_ds_converted_" + method_name + ".csv");
    std::ofstream ds_spectra_file("output/comparsion_ds.csv");

    int successful = 0;
    std::vector<Float> losses;
    std::vector<Float> spec_maes(in_spectra.size());
    std::vector<Float> spec_sams(in_spectra.size());

    for(unsigned i = 0; i < in_spectra.size(); ++i) {
        vec3 xyz = spectre2xyz(in_spectra[i]);
        vec3 rgb = xyz2rgb(xyz);
        ISpectrum::ptr spec = upsampler.upsample_pixel(Pixel::from_vec3(rgb));


        vec3 lab = xyz2cielab(spectre2xyz(*spec));
        const Float deltaE = vec3::distance(lab, xyz2cielab(xyz));
        successful += deltaE < 2.333f;
        losses.push_back(deltaE);

        spec_maes[i] = metrics::mae(*spec, in_spectra[i], wavelenghts);
        spec_sams[i] = metrics::sam(*spec, in_spectra[i], wavelenghts);
        out_spectra[i] = util::convert_to_spd(*spec);

    }

    Float de_mean = std::accumulate(losses.begin(), losses.end(), 0.0f) / in_spectra.size();
    Float mae_mean = std::accumulate(spec_maes.begin(), spec_maes.end(), 0.0f) / in_spectra.size();
    Float sam_mean = std::accumulate(spec_sams.begin(), spec_sams.end(), 0.0f) / in_spectra.size();

    //auto [minde, maxde] = minmax_element(losses.begin(), losses.end());
   //auto [minsam, maxsam] = minmax_element(spec_sams.begin(), spec_sams.end());

    output_file << std::setprecision(8)
                << "# Average DeltaE loss: " << de_mean << ";\n"
                //<< "# Min DeltaE: " <<  *minde << ", max DeltaE: " << *maxde << "\n"
                << "# DeltaE stddev: " << stddev(losses, de_mean) << ";\n"
                << "# Unrecognizable difference in " << successful << "/" << in_spectra.size() << " values.\n#" << std::endl;

    output_file << "# Average spectra MAE is " << mae_mean << ", stddev: " << stddev(spec_maes, mae_mean) << ";\n"
                << "# Average spectra SAM is " << sam_mean << ", stddev: " << stddev(spec_sams, sam_mean) << ";\n"
               // << "# Min SAM: " << *minsam << ", max SAM: " << *maxsam << ".\n#\n"
                << "DeltaE,Spectral MAE,SAM" << std::endl;

    for(unsigned i = 0; i < in_spectra.size(); ++i) {
        output_file << losses[i] << "," << spec_maes[i] << "," << spec_sams[i] << "\n";
    }
    output_file.flush();
    output_file.close();

    result_spectra_file << std::setprecision(16);
    ds_spectra_file << std::setprecision(16);
    for(int i = WAVELENGHTS_START; i < WAVELENGHTS_END; i += WAVELENGHTS_STEP) {
        result_spectra_file << i << ",";
        ds_spectra_file << i << ",";
    }
    result_spectra_file << WAVELENGHTS_END << std::endl;
    ds_spectra_file << WAVELENGHTS_END << std::endl;

    for(const BasicSpectrum &sp : in_spectra) {
        for(int i = WAVELENGHTS_START; i < WAVELENGHTS_END; i += WAVELENGHTS_STEP) {
            ds_spectra_file << sp(i) << ",";
        }
        ds_spectra_file << sp(WAVELENGHTS_END) << std::endl;
    }

    for(const BasicSpectrum &sp : out_spectra) {
        for(int i = WAVELENGHTS_START; i < WAVELENGHTS_END; i += WAVELENGHTS_STEP) {
            result_spectra_file << sp(i) << ",";
        }
        result_spectra_file << sp(WAVELENGHTS_END) << std::endl;
    }
}

    
void img_reupsample(const IUpsampler &upsampler, const std::string &path, const std::string &method)
{
    const std::string name = fs::path(path).stem();
    std::cout << name << " " << method << std::endl;

    ISpectralImage::ptr spec_img_gt;
    ISpectrum::csptr light_source;
    if(!util::load_spectral_image(path, spec_img_gt, light_source)) throw std::runtime_error("Failed to load");

    std::cout << "Loaded image" << std::endl;

    const unsigned width = spec_img_gt->get_width();
    const unsigned height = spec_img_gt->get_height();

    std::ofstream output_file("output/comparsion/im_result_" + name + "_" + method + ".txt");

    const Image img_rgb_gt = spectral_image2rgb(*spec_img_gt);

    std::cout << "Downsampled loaded image" << std::endl;

    Image img_rgb_reupsampled;

    {   
        auto t1 = high_resolution_clock::now();
        ISpectralImage::ptr spec_img_reupsampled = upsampler.upsample(img_rgb_gt);
        auto t2 = high_resolution_clock::now();
        std::cout << "Reupsampled image" << std::endl;

        output_file << "Upsampling time: " << duration_cast<std::chrono::milliseconds>(t2 - t1).count() << std::endl;

        std::vector<Float> maes;
        std::vector<Float> sams;
        maes.reserve(width * height);
        sams.reserve(width * height);

        //#pragma omp parallel for reduction(+:mae_sum,sam_sum)
        for(unsigned j = 0; j < height; ++j) {
            for(unsigned i = 0; i < width; ++i) {
                maes.push_back(metrics::mae(spec_img_reupsampled->at(i, j), spec_img_gt->at(i, j)));
                sams.push_back(metrics::sam(spec_img_reupsampled->at(i, j), spec_img_gt->at(i, j)));
            }
        } 

        Float mae_mean = Float(std::accumulate(maes.begin(), maes.end(), 0.0) / double(width * height));
        Float sam_mean = Float(std::accumulate(sams.begin(), sams.end(), 0.0) / double(width * height));

        output_file << "Average spectra MAE: " << mae_mean << ", stddev = " << stddev(maes, mae_mean) << ";\n"
                    << "Average SAM: " << sam_mean << ", stddev = " << stddev(sams, sam_mean) << std::endl;

        std::cout << "Metrics calculated" << std::endl;

        img_rgb_reupsampled = spectral_image2rgb(*spec_img_reupsampled);
        std::cout << "Redownsampled image" << std::endl;
    }

    img_rgb_gt.save(format("output/comparsion/im_%s_downsampled.png", name.c_str()));
    img_rgb_reupsampled.save(format("output/comparsion/im_%s_redownsampled_%s.png", name.c_str(), method.c_str()));

}

void img_simpletest(const IUpsampler &upsampler, const std::string &path, const std::string &method)
{
    Image image_gt{path};
    const std::string name = fs::path(path).stem();
    std::ofstream output_file("output/comparsion/textures/result_" + name + "_" + method + ".txt");

    std::cout << "Upsampling " << path << std::endl;
    auto t1 = high_resolution_clock::now();
    ISpectralImage::ptr spectral_img = upsampler.upsample(image_gt);
    auto t2 = high_resolution_clock::now();
    output_file << "Upsampling took " << duration_cast<std::chrono::milliseconds>(t2 - t1).count() << " ms." << std::endl;

    std::cout << "Downsampling" << std::endl;
    Image image = spectral_image2rgb(*spectral_img);
    image.save("output/comparsion/textures/" + name + "_" + method + ".png");
}

vec3 ior2rgb(const BasicSpectrum &spec_eta, const BasicSpectrum &spec_k)
{
    Float cieyint = util::get_cie_y_integral();
    vec3 xyz{0.0f, 0.0f, 0.0f};

    unsigned idx = 0u;
    for(int lambda = WAVELENGHTS_START; lambda <= WAVELENGHTS_END; lambda += WAVELENGHTS_STEP) {
        const Float eta = spec_eta(lambda);
        const Float k = spec_k(lambda);

        const Float k2 = k * k;
        const Float val_lv = (((eta - 1.0f) * (eta - 1.0f) + k2) / ((eta + 1.0f) * (eta + 1.0f) + k2)) * util::CIE_D6500.get_or_interpolate(lambda);
        
        xyz.x += X_CURVE[idx] * val_lv;
        xyz.y += Y_CURVE[idx] * val_lv;
        xyz.z += Z_CURVE[idx] * val_lv;
        idx += 1;
    }
    xyz /= cieyint;
    return xyz2rgb(xyz);
}

void ior_reupsample(const IUpsampler &upsampler, const std::string &path_eta, const std::string &path_k, const std::string &method)
{
    const BasicSpectrum spec_eta_gt = util::load_spd(path_eta);
    const BasicSpectrum spec_k_gt = util::load_spd(path_k);

    const vec3 rgb_gt = ior2rgb(spec_eta_gt, spec_k_gt);

    std::cout << "Color is " << (rgb_gt * 255.0f).cast<int>() << std::endl;

    ISpectrum::ptr spectrum = upsampler.upsample_pixel(Pixel::from_vec3(rgb_gt));

    BasicSpectrum spec_eta;
    BasicSpectrum spec_k;

    for(int wl = WAVELENGHTS_START; wl <= WAVELENGHTS_END; wl += WAVELENGHTS_STEP) {
        auto [eta, k] = color2ior(spectrum->get_or_interpolate(wl));
        spec_eta.set(wl, eta);
        spec_k.set(wl, k);
    }

    const vec3 rgb = ior2rgb(spec_eta, spec_k);


    std::cout << "eta MAE: " << metrics::mae(spec_eta_gt, spec_eta) << ";\n"
              << "eta SAM: " << metrics::sam(spec_eta_gt, spec_eta) << ";\n"
              << "k MAE: " << metrics::mae(spec_k_gt, spec_k) << ";\n"
              << "k SAM: " << metrics::sam(spec_k_gt, spec_k) << ";\n"
              << "DeltaE: " << metrics::cie_delta_e(rgb_gt, rgb) << ";\n"
              << (rgb * 255.0f).cast<int>() << std::endl;
    (void) method;

}

const vec3 COLOR_POWER{27.4722f, 71.8074f, 7.63813f};

void emiss_reupsample()
{
    std::vector<BasicSpectrum> in_spectra;
    std::vector<Float> wavelenghts;
    std::vector<vec3> in_rgbs;
    load_spec_ds("output/dataset_spectra_val.csv", wavelenghts, in_spectra);
    load_vec_ds("output/dataset_rgb_val.csv", in_rgbs);
    std::vector<BasicSpectrum> out_spectra(in_spectra.size());

    std::ofstream output_file("output/comparsion/comparsion_em_result.csv");
    std::ofstream result_spectra_file("output/comparsion/comparsion_em_converted.csv");
    std::ofstream ds_spectra_file("output/comparsion/comparsion_em.csv");

    int successful = 0;
    std::vector<Float> losses;
    std::vector<Float> spec_maes(in_spectra.size());
    std::vector<Float> spec_sams(in_spectra.size());


    std::ifstream lut_file{"resources/f_emission_lut.eflf"};
    FourierLUT lut = FourierLUT::load_from(lut_file);
    lut_file.close();

    for(unsigned i = 0; i < in_spectra.size(); ++i) {
        vec3 rgb = in_rgbs[i];

        vec3 rgb_norm = rgb / rgb.max();
        Float power = 25.0f * rgb.max();
       // Float base_power = (rgb * COLOR_POWER).sum();
        FourierEmissionSpectrum spec = upsample::fourier_emiss(rgb_norm, power, lut);
        Float illum = util::get_cie_y_integral(spec);

        vec3 lab = xyz2cielab(spectre2xyz0(spec) / (illum));
        const Float deltaE = vec3::distance(lab, rgb2cielab(rgb_norm));
        successful += deltaE < 2.333f;
        losses.push_back(deltaE);

        spec_maes[i] = metrics::mae(spec, in_spectra[i], wavelenghts);
        spec_sams[i] = metrics::sam(spec, in_spectra[i], wavelenghts);
        out_spectra[i] = util::convert_to_spd(spec);

    }

    Float de_mean = std::accumulate(losses.begin(), losses.end(), 0.0f) / in_spectra.size();
    Float mae_mean = std::accumulate(spec_maes.begin(), spec_maes.end(), 0.0f) / in_spectra.size();
    Float sam_mean = std::accumulate(spec_sams.begin(), spec_sams.end(), 0.0f) / in_spectra.size();

    //auto [minde, maxde] = minmax_element(losses.begin(), losses.end());
   //auto [minsam, maxsam] = minmax_element(spec_sams.begin(), spec_sams.end());

    output_file << std::setprecision(8)
                << "# Average DeltaE loss: " << de_mean << ";\n"
                //<< "# Min DeltaE: " <<  *minde << ", max DeltaE: " << *maxde << "\n"
                << "# DeltaE stddev: " << stddev(losses, de_mean) << ";\n"
                << "# Unrecognizable difference in " << successful << "/" << in_spectra.size() << " values.\n#" << std::endl;

    output_file << "# Average spectra MAE is " << mae_mean << ", stddev: " << stddev(spec_maes, mae_mean) << ";\n"
                << "# Average spectra SAM is " << sam_mean << ", stddev: " << stddev(spec_sams, sam_mean) << ";\n"
               // << "# Min SAM: " << *minsam << ", max SAM: " << *maxsam << ".\n#\n"
                << "DeltaE,Spectral MAE,SAM" << std::endl;

    for(unsigned i = 0; i < in_spectra.size(); ++i) {
        output_file << losses[i] << "," << spec_maes[i] << "," << spec_sams[i] << "\n";
    }
    output_file.flush();
    output_file.close();

    result_spectra_file << std::setprecision(16);
    ds_spectra_file << std::setprecision(16);
    for(int i = WAVELENGHTS_START; i < WAVELENGHTS_END; i += WAVELENGHTS_STEP) {
        result_spectra_file << i << ",";
        ds_spectra_file << i << ",";
    }
    result_spectra_file << WAVELENGHTS_END << std::endl;
    ds_spectra_file << WAVELENGHTS_END << std::endl;

    for(const BasicSpectrum &sp : in_spectra) {
        for(int i = WAVELENGHTS_START; i < WAVELENGHTS_END; i += WAVELENGHTS_STEP) {
            ds_spectra_file << sp(i) << ",";
        }
        ds_spectra_file << sp(WAVELENGHTS_END) << std::endl;
    }

    for(const BasicSpectrum &sp : out_spectra) {
        for(int i = WAVELENGHTS_START; i < WAVELENGHTS_END; i += WAVELENGHTS_STEP) {
            result_spectra_file << sp(i) << ",";
        }
        result_spectra_file << sp(WAVELENGHTS_END) << std::endl;
    }
}

int main(int argc, char **argv)
{
    if(argc < 2) return 1;
    const std::string method = argv[1];
    if(method == "fourier") {
        emiss_reupsample();
        return 0;
    }
    std::unique_ptr<IUpsampler> upsampler = get_upsampler_by_name(method);
  
    if(!strcmp(argv[2], "ds")) {
        dataset_reupsample(*upsampler, method);
        return 0;
    }
    else if(!strcmp(argv[2], "si")) {
        if(argc >= 4) {
            img_simpletest(*upsampler, std::string(argv[3]), method);
            return 0;
        }
    }
    else if(!strcmp(argv[2], "im")) {
        if(argc >= 4) {
            auto t1 = high_resolution_clock::now();
            img_reupsample(*upsampler, std::string(argv[3]), method);
            auto t2 = high_resolution_clock::now();
            std::cout << "Time: " << duration_cast<std::chrono::milliseconds>(t2 - t1).count() << std::endl;
            return 0;
        }
    }
    else if(!strcmp(argv[2], "ior")) {
        if(argc >= 5) {
            ior_reupsample(*upsampler, argv[3], argv[4], method);
            return 0;
        }
    }
    return 1;
} 
