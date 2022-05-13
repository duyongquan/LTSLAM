#include "xslam/vins/feature_tracker/feature_tracker_options.h"

namespace xslam {
namespace vins {
namespace feature_tracker {

proto::FeatureTrackerOptions CreateFeatureTrackerOptions(
    common::LuaParameterDictionary* const lua_parameter_dictionary)
{
    proto::FeatureTrackerOptions options;
    // camera calibration 
    options.set_model_type(lua_parameter_dictionary->GetString("model_type"));
    options.set_camera_name(lua_parameter_dictionary->GetString("camera_name"));
    options.set_image_width(lua_parameter_dictionary->GetInt("image_width"));
    options.set_image_height(lua_parameter_dictionary->GetInt("image_height"));

    // distortion
    options.set_k1(lua_parameter_dictionary->GetDouble("k1"));
    options.set_k2(lua_parameter_dictionary->GetDouble("k2"));
    options.set_p1(lua_parameter_dictionary->GetDouble("p1"));
    options.set_p2(lua_parameter_dictionary->GetDouble("p2"));

    // projection
    options.set_fx(lua_parameter_dictionary->GetDouble("fx"));
    options.set_fy(lua_parameter_dictionary->GetDouble("fy"));
    options.set_cx(lua_parameter_dictionary->GetDouble("cx"));
    options.set_cy(lua_parameter_dictionary->GetDouble("cy"));

    // extrinsic
    options.set_estimate_extrinsic(lua_parameter_dictionary->GetInt("estimate_extrinsic")); 

    // unsynchronization parameters
    options.set_estimate_td(lua_parameter_dictionary->GetInt("estimate_td")); 
    options.set_td(lua_parameter_dictionary->GetDouble("td"));

    // rolling shutter parameters
    options.set_rolling_shutter(lua_parameter_dictionary->GetInt("rolling_shutter")); 
    options.set_rolling_shutter_tr(lua_parameter_dictionary->GetInt("rolling_shutter_tr")); 

    // feature traker paprameters
    options.set_max_cnt(lua_parameter_dictionary->GetInt("max_cnt")); 
    options.set_min_distance(lua_parameter_dictionary->GetInt("min_dist")); 
    options.set_freq(lua_parameter_dictionary->GetInt("freq")); 

    options.set_f_threshold(lua_parameter_dictionary->GetDouble("F_threshold"));
    options.set_show_track(lua_parameter_dictionary->GetBool("show_track")); 
    options.set_equalize(lua_parameter_dictionary->GetBool("equalize")); 
    options.set_fisheye(lua_parameter_dictionary->GetBool("fisheye"));     

    // Debug 
    VinsOptionsDebugToString(options);
    return options;
}

void VinsOptionsDebugToString(const proto::FeatureTrackerOptions& options)
{
    LOG(INFO) << "[FeatureTrackerOptions :]";
    // camera calibration
    LOG(INFO) << "  camera calibration :";
    LOG(INFO) << "     model_type: " << options.model_type();
    LOG(INFO) << "     camera_name: " << options.camera_name();
    LOG(INFO) << "     image_width: " << options.image_width();
    LOG(INFO) << "     image_height: " << options.image_height();

    // distortion
    LOG(INFO) << "  distortion :";
    LOG(INFO) << "     k1: " << options.k1();
    LOG(INFO) << "     k2: " << options.k2();
    LOG(INFO) << "     p1: " << options.p1();
    LOG(INFO) << "     p2: " << options.p2();


    // projection
    LOG(INFO) << "  projection :";
    LOG(INFO) << "     fx: " << options.fx();
    LOG(INFO) << "     fy: " << options.fy();
    LOG(INFO) << "     cx: " << options.cx();
    LOG(INFO) << "     cy: " << options.cy();

    // extrinsic
    LOG(INFO) << "  extrinsic :";
    LOG(INFO) << "     estimate_extrinsic: " << options.estimate_extrinsic();

    // unsynchronization parameters
    LOG(INFO) << "  unsynchronization :";
    LOG(INFO) << "     estimate_td: " << options.estimate_td();
    LOG(INFO) << "     td: " << options.td();

    // rolling shutter parameters
    LOG(INFO) << "  rolling shutter :";
    LOG(INFO) << "     rolling_shutter: " << options.rolling_shutter();
    LOG(INFO) << "     rolling_shutter_tr: " << options.rolling_shutter_tr();

    // feature tracker paprameters
    LOG(INFO) << "  feature tracker :";
    LOG(INFO) << "     max_cnt: " << options.max_cnt();
    LOG(INFO) << "     min_dist: " << options.min_distance();
    LOG(INFO) << "     freq: " << options.freq();
    LOG(INFO) << "     F_threshold: " << options.f_threshold();
    LOG(INFO) << "     show_track: " << options.show_track();
    LOG(INFO) << "     equalize: " << options.equalize();
    LOG(INFO) << "     fisheye: " << options.fisheye();
}

} // namespace feature_tracker
} // namespace vins
} // namespace xslam 