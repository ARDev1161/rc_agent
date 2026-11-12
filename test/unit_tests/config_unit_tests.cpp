#include "../../config/configprocessor.h"
#include "../../config/endpoint_bindings.h"
#include <stdexcept>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>

namespace fs = std::filesystem;

// Tests for readConfig function
class readConfig : public ::testing::Test {
protected:
  ConfigProcessor *config;
  fs::path filepath;

  void SetUp() {
    filepath = fs::temp_directory_path();
    filepath.append("amur_test_config_read");
    if (fs::exists(filepath))
      fs::remove(filepath);

    config = new ConfigProcessor(filepath);
  }

  void TearDown() {
    fs::remove(filepath);
    filepath.clear();
    delete config;
  }
};

TEST_F(readConfig, emptyFilename) {
  // Test when config filename is empty. Expectation of failure.
  ASSERT_EQ(EXIT_FAILURE, config->readConfig(""));

  config->setConfigFilename("");
  ASSERT_EQ(EXIT_FAILURE, config->readConfig());
}

TEST_F(readConfig, notExist) {
  // Test when config file are not exist. Expectation of failure.
  ASSERT_EQ(EXIT_FAILURE, config->readConfig(filepath));

  config->setConfigFilename(filepath);
  ASSERT_EQ(EXIT_FAILURE, config->readConfig());
}

TEST_F(readConfig, exist) {
  // Test when config file are exist. Expectation of success.
  std::ofstream outfile(filepath);
  outfile << "" << std::endl;

  ASSERT_EQ(EXIT_SUCCESS, config->readConfig(filepath));

  config->setConfigFilename(filepath);
  ASSERT_EQ(EXIT_SUCCESS, config->readConfig());
}

TEST_F(readConfig, wrongFormat) {
  // Test when config file have wrong content. Expectation of success.
  std::ofstream outfile(filepath);
  outfile << "test \"string\" " << std::endl;

  ASSERT_EQ(EXIT_FAILURE, config->readConfig(filepath));

  config->setConfigFilename(filepath);
  ASSERT_EQ(EXIT_FAILURE, config->readConfig());
}

// Tests for writeConfig function
class writeConfig : public ::testing::Test {
protected:
  ConfigProcessor *config;
  fs::path filepath;

  void SetUp() {}

  void TearDown() {
    filepath.clear();
    delete config;
  }
};

TEST_F(writeConfig, wrongPath) {
  // Restricted symbols in path
  config = new ConfigProcessor("amur_test_config_write_!@#$%^&*()|/.,");
  ASSERT_EQ(EXIT_FAILURE, config->writeConfig());
}

TEST_F(writeConfig, emptyPath) {
  // Empty path
  config = new ConfigProcessor("");
  ASSERT_EQ(EXIT_FAILURE, config->writeConfig());
}

TEST_F(writeConfig, existedFile) {
  // Test write config when file does exist. Expectation of success.
  filepath = fs::temp_directory_path();
  filepath.append("amur_test_config_write");
  if (fs::exists(filepath))
    fs::remove(filepath);

  std::ofstream outfile(filepath);
  outfile << "test = \"string\"; " << std::endl;

  config = new ConfigProcessor(filepath);

  config->rewriteString("test", "new-value");
  ASSERT_EQ(EXIT_SUCCESS, config->writeConfig());
  fs::remove(filepath);
}

TEST_F(writeConfig, removedFile) {
  // Test write config when file does not exist. Expectation of success.
  filepath = fs::temp_directory_path();
  filepath.append("amur_test_config_write");
  if (fs::exists(filepath))
    fs::remove(filepath);

  config = new ConfigProcessor(filepath);

  ASSERT_EQ(EXIT_SUCCESS, config->writeConfig());
  fs::remove(filepath);
}

// Tests for isScalar, isNumber & isAggregate functions
class isType : public ::testing::Test {
protected:
  ConfigProcessor *config;
  fs::path filepath;

  void SetUp() {
    filepath = fs::temp_directory_path();
    filepath.append("amur_test_config_istype");
    if (fs::exists(filepath))
      fs::remove(filepath);

    std::ofstream outfile(filepath);
    outfile << "aggregate:" << std::endl
            << "{" << std::endl
            << "scalar = \"int float bool string \"; " << std::endl
            << "number = 42;" << std::endl
            << "};" << std::endl;

    config = new ConfigProcessor(filepath);
  }

  void TearDown() {
    fs::remove(filepath);
    filepath.clear();
    delete config;
  }
};

TEST_F(isType, emptySettingName) {
  // Test with empty setting name
  ASSERT_EQ(libconfig::Setting::TypeNone, config->isNumber(""));
  ASSERT_EQ(libconfig::Setting::TypeNone, config->isScalar(""));
  ASSERT_EQ(libconfig::Setting::TypeNone, config->isAggregate(""));
}

TEST_F(isType, noSetting) {
  // Test for non existed setting
  ASSERT_EQ(libconfig::Setting::TypeNone, config->isNumber("nonexist"));
  ASSERT_EQ(libconfig::Setting::TypeNone, config->isScalar("nonexist"));
  ASSERT_EQ(libconfig::Setting::TypeNone, config->isAggregate("nonexist"));
}

TEST_F(isType, checkScalar) {
  ASSERT_EQ(true, config->isScalar("aggregate.scalar")); // Test if scalar
}

TEST_F(isType, checkNonScalar) {
  ASSERT_EQ(false, config->isScalar("aggregate")); // Test if not scalar
}

TEST_F(isType, checkNumber) {
  ASSERT_EQ(true, config->isNumber("aggregate.number")); // Test if number
}

TEST_F(isType, checkNonNumber) {
  ASSERT_EQ(false, config->isNumber("aggregate")); // Test if not number
}

TEST_F(isType, checkAggregate) {
  ASSERT_EQ(true, config->isAggregate("aggregate")); // Test if aggregate
}

TEST_F(isType, checkNonAggregate) {
  ASSERT_EQ(false,
            config->isAggregate("aggregate.number")); // Test if not aggregate
}

// Tests for getType function
class getType : public ::testing::Test {
protected:
  ConfigProcessor *config;
  fs::path filepath;

  void SetUp() {
    filepath = fs::temp_directory_path();
    filepath.append("amur_test_config_gettype");
    if (fs::exists(filepath))
      fs::remove(filepath);

    std::ofstream outfile(filepath);
    outfile << "group:" << std::endl
            << "{" << std::endl
            << "bool = true;" << std::endl
            << "float = 0.42;" << std::endl
            << "int = 42;" << std::endl
            << "int64 = 42L;" << std::endl
            << "string = \"usual string \"; " << std::endl
            << "};" << std::endl
            << "array = [ ]; " << std::endl
            << "list = ( ); " << std::endl;

    config = new ConfigProcessor(filepath);
  }

  void TearDown() {
    fs::remove(filepath);
    filepath.clear();
    delete config;
  }
};

TEST_F(getType, emptySettingName) {
  ASSERT_EQ(libconfig::Setting::TypeNone, config->getType(""));
}

TEST_F(getType, checkBool) {
  ASSERT_EQ(libconfig::Setting::TypeBoolean, config->getType("group.bool"));
}

TEST_F(getType, checkFloat) {
  ASSERT_EQ(libconfig::Setting::TypeFloat, config->getType("group.float"));
}

TEST_F(getType, checkInt) {
  ASSERT_EQ(libconfig::Setting::TypeInt, config->getType("group.int"));
}

TEST_F(getType, checkInt64) {
  ASSERT_EQ(libconfig::Setting::TypeInt64, config->getType("group.int64"));
}

TEST_F(getType, checkString) {
  ASSERT_EQ(libconfig::Setting::TypeString, config->getType("group.string"));
}

TEST_F(getType, checkGroup) {
  ASSERT_EQ(libconfig::Setting::TypeGroup, config->getType("group"));
}

TEST_F(getType, checkArray) {
  ASSERT_EQ(libconfig::Setting::TypeArray, config->getType("array"));
}

TEST_F(getType, checkList) {
  ASSERT_EQ(libconfig::Setting::TypeList, config->getType("list"));
}

// Tests for search functions
class searchSetting : public ::testing::Test {
protected:
  ConfigProcessor *config;
  fs::path filepath;

  std::string outString;
  int outInt;
  float outFloat;
  bool outBool;

  void SetUp() {
    filepath = fs::temp_directory_path();
    filepath.append("amur_test_config_gettype");
    if (fs::exists(filepath))
      fs::remove(filepath);

    std::ofstream outfile(filepath);
    outfile << "group:" << std::endl
            << "{" << std::endl
            << "bool = true;" << std::endl
            << "float = 0.42;" << std::endl
            << "int = 42;" << std::endl
            << "int64 = 42L;" << std::endl
            << "string = \"usual string\"; " << std::endl
            << "};" << std::endl
            << "array = [ ]; " << std::endl
            << "list = ( ); " << std::endl;

    config = new ConfigProcessor(filepath);
  }

  void TearDown() {
    outBool = false;
    outFloat = 0.0;
    outString = "";
    outInt = 0;

    fs::remove(filepath);
    filepath.clear();
    delete config;
  }
};

TEST_F(searchSetting, emptySettingName) {
  ASSERT_EQ(EXIT_FAILURE, config->searchString("", outString));
  ASSERT_EQ(EXIT_FAILURE, config->searchInt("", outInt));
  ASSERT_EQ(EXIT_FAILURE, config->searchFloat("", outFloat));
  ASSERT_EQ(EXIT_FAILURE, config->searchBool("", outBool));
}

TEST_F(searchSetting, nonexistedName) {
  ASSERT_EQ(EXIT_FAILURE, config->searchString("nonexist", outString));
  ASSERT_EQ(EXIT_FAILURE, config->searchInt("nonexist", outInt));
  ASSERT_EQ(EXIT_FAILURE, config->searchFloat("nonexist", outFloat));
  ASSERT_EQ(EXIT_FAILURE, config->searchBool("nonexist", outBool));
}

TEST_F(searchSetting, searchString) {
  ASSERT_EQ(EXIT_SUCCESS, config->searchString("group.string", outString));
  ASSERT_EQ("usual string", outString);
}

TEST_F(searchSetting, searchInt) {
  ASSERT_EQ(EXIT_SUCCESS, config->searchInt("group.int", outInt));
  ASSERT_EQ(42, outInt);
}

TEST_F(searchSetting, searchFloat) {
  float res = 0.42;
  ASSERT_EQ(EXIT_SUCCESS, config->searchFloat("group.float", outFloat));
  ASSERT_EQ(res, outFloat);
}

TEST_F(searchSetting, searchBool) {
  ASSERT_EQ(EXIT_SUCCESS, config->searchBool("group.bool", outBool));
  ASSERT_EQ(true, outBool);
}

// Tests for rewrite functions
class rewriteSetting : public ::testing::Test {
protected:
  ConfigProcessor *config;
  fs::path filepath;

  std::string valueString = "string";
  int valueInt = 42;
  float valueFloat = 0.42;
  bool valueBool = true;

  void SetUp() {
    filepath = fs::temp_directory_path();
    filepath.append("amur_test_config_gettype");
    if (fs::exists(filepath))
      fs::remove(filepath);

    std::ofstream outfile(filepath);
    outfile << "group:" << std::endl
            << "{" << std::endl
            << "bool = true;" << std::endl
            << "float = 0.42;" << std::endl
            << "int = 42;" << std::endl
            << "int64 = 42L;" << std::endl
            << "string = \"usual string\"; " << std::endl
            << "};" << std::endl
            << "array = [ ]; " << std::endl
            << "list = ( ); " << std::endl;

    config = new ConfigProcessor(filepath);
  }

  void TearDown() {
    fs::remove(filepath);
    filepath.clear();
    delete config;
  }
};

TEST_F(rewriteSetting, emptySettingName) {
  ASSERT_EQ(EXIT_FAILURE, config->rewriteString("", valueString));
  ASSERT_EQ(EXIT_FAILURE, config->searchInt("", valueInt));
  ASSERT_EQ(EXIT_FAILURE, config->searchFloat("", valueFloat));
  ASSERT_EQ(EXIT_FAILURE, config->searchBool("", valueBool));
}

TEST_F(rewriteSetting, notExisted) {
  ASSERT_EQ(EXIT_FAILURE, config->rewriteString("nonexist", valueString));
  ASSERT_EQ(EXIT_FAILURE, config->searchInt("nonexist", valueInt));
  ASSERT_EQ(EXIT_FAILURE, config->searchFloat("nonexist", valueFloat));
  ASSERT_EQ(EXIT_FAILURE, config->searchBool("nonexist", valueBool));
}

TEST_F(rewriteSetting, nonString) {
  ASSERT_EQ(EXIT_FAILURE, config->rewriteString("group", valueString));
}

TEST_F(rewriteSetting, String) {
  std::string tmp;
  ASSERT_EQ(EXIT_SUCCESS, config->rewriteString("group.string", valueString));
  config->searchString("group.string", tmp);
  ASSERT_EQ(valueString, tmp);
}

TEST_F(rewriteSetting, nonInteger) {
  ASSERT_EQ(EXIT_FAILURE, config->rewriteString("group", valueString));
}

TEST_F(rewriteSetting, Integer) {
  int tmp;
  ASSERT_EQ(EXIT_SUCCESS, config->rewriteInt("group.int", valueInt));
  config->searchInt("group.int", tmp);
  ASSERT_EQ(valueInt, tmp);
}

TEST_F(rewriteSetting, nonFloat) {
  ASSERT_EQ(EXIT_FAILURE, config->rewriteString("group", valueString));
}

TEST_F(rewriteSetting, Float) {
  float tmp;
  ASSERT_EQ(EXIT_SUCCESS, config->rewriteFloat("group.float", valueFloat));
  config->searchFloat("group.float", tmp);
  ASSERT_EQ(valueFloat, tmp);
}

TEST_F(rewriteSetting, nonBool) {
  ASSERT_EQ(EXIT_FAILURE, config->rewriteString("group", valueString));
}

TEST_F(rewriteSetting, Bool) {
  bool tmp;
  ASSERT_EQ(EXIT_SUCCESS, config->rewriteBool("group.bool", valueBool));
  config->searchBool("group.bool", tmp);
  ASSERT_EQ(valueBool, tmp);
}

// Tests for getConfigName & setConfigName functions
TEST(getConfigName, setAndGet) {
  ConfigProcessor config;
  config.setConfigFilename("test_filename_config");
  ASSERT_EQ("test_filename_config", config.getConfigFilename());
}

// int main(int argc, char **argv)
//{
//     ::testing::InitGoogleTest(&argc, argv);

//    return RUN_ALL_TESTS();
//}

// rc-api-v2: validate libconfig string lists and fail without partial output.
TEST(ConfigStringList, ReadsCapabilities) {
  const auto path = fs::temp_directory_path() / "rc_agent_v2_capabilities_test.cfg";
  {
    std::ofstream out(path);
    out << "api: { capabilities = (\"manual.velocity\", \"navigation.nav2\"); };" << std::endl;
  }
  ConfigProcessor config(path.string());
  std::vector<std::string> result;
  EXPECT_EQ(EXIT_SUCCESS, config.searchStringList("api.capabilities", result));
  EXPECT_EQ((std::vector<std::string>{"manual.velocity", "navigation.nav2"}), result);
  fs::remove(path);
}

TEST(ConfigStringList, RejectsNonStringWithoutPartialOutput) {
  const auto path = fs::temp_directory_path() / "rc_agent_v2_invalid_list_test.cfg";
  {
    std::ofstream out(path);
    out << "api: { capabilities = (\"manual.velocity\", 42); scalar = \"x\"; };" << std::endl;
  }
  ConfigProcessor config(path.string());
  std::vector<std::string> result{"keep"};
  EXPECT_EQ(EXIT_FAILURE, config.searchStringList("api.capabilities", result));
  EXPECT_EQ((std::vector<std::string>{"keep"}), result);
  EXPECT_EQ(EXIT_FAILURE, config.searchStringList("api.scalar", result));
  EXPECT_EQ(EXIT_FAILURE, config.searchStringList("api.missing", result));
  fs::remove(path);
}


TEST(ConfigMalformedScalars, RejectWrongTypeWithoutAbortingAgent) {
  const auto path = fs::temp_directory_path() / "rc_agent_v2_wrong_scalar_types.cfg";
  {
    std::ofstream out(path);
    out << "Amur: { API: { robot_id = 123; display_name = true; }; };"
           " wrong: { text = \"not-a-number\"; };" << std::endl;
  }
  ConfigProcessor config(path.string());
  std::string name = "unchanged";
  int number = 42;
  float value = 1.25F;
  bool enabled = true;
  EXPECT_EQ(EXIT_FAILURE, config.searchString("Amur.API.robot_id", name));
  EXPECT_EQ("unchanged", name);
  EXPECT_EQ(EXIT_FAILURE, config.searchString("Amur.API.display_name", name));
  EXPECT_EQ("unchanged", name);
  EXPECT_EQ(EXIT_FAILURE, config.searchInt("wrong.text", number));
  EXPECT_EQ(42, number);
  EXPECT_EQ(EXIT_FAILURE, config.searchFloat("wrong.text", value));
  EXPECT_FLOAT_EQ(1.25F, value);
  EXPECT_EQ(EXIT_FAILURE, config.searchBool("wrong.text", enabled));
  EXPECT_TRUE(enabled);
  fs::remove(path);
}


// P2: distinct robot profiles retain their configured endpoint strings;
// malformed endpoint types must not overwrite a previously valid value.
TEST(ConfigEndpointProfiles, IndependentBindingsAndMalformedType) {
  const auto path_a = fs::temp_directory_path() / "rc_agent_v2_profile_a_endpoints.cfg";
  const auto path_b = fs::temp_directory_path() / "rc_agent_v2_profile_b_endpoints.cfg";
  {
    std::ofstream out(path_a);
    out << "Amur: { API: { endpoints: { odom_topic = \"/alpha/odom\";"
           " cmd_vel_topic = \"/alpha/cmd_vel\";"
           " navigate_to_pose_action = \"/alpha/navigate_to_pose\"; }; }; };\n";
  }
  {
    std::ofstream out(path_b);
    out << "Amur: { API: { endpoints: { odom_topic = \"/beta/odom\";"
           " cmd_vel_topic = 17;"
           " navigate_to_pose_action = \"/beta/navigate_to_pose\"; }; }; };\n";
  }
  ConfigProcessor a(path_a.string());
  ConfigProcessor b(path_b.string());
  std::string value;
  EXPECT_EQ(EXIT_SUCCESS, a.searchString("Amur.API.endpoints.odom_topic", value));
  EXPECT_EQ("/alpha/odom", value);
  EXPECT_EQ(EXIT_SUCCESS, a.searchString("Amur.API.endpoints.cmd_vel_topic", value));
  EXPECT_EQ("/alpha/cmd_vel", value);
  EXPECT_EQ(EXIT_SUCCESS, a.searchString("Amur.API.endpoints.navigate_to_pose_action", value));
  EXPECT_EQ("/alpha/navigate_to_pose", value);
  EXPECT_EQ(EXIT_SUCCESS, b.searchString("Amur.API.endpoints.odom_topic", value));
  EXPECT_EQ("/beta/odom", value);
  EXPECT_EQ(EXIT_FAILURE, b.searchString("Amur.API.endpoints.cmd_vel_topic", value));
  EXPECT_EQ("/beta/odom", value);
  EXPECT_EQ(EXIT_SUCCESS, b.searchString("Amur.API.endpoints.navigate_to_pose_action", value));
  EXPECT_EQ("/beta/navigate_to_pose", value);
  fs::remove(path_a);
  fs::remove(path_b);
}

// P2: exercise the exact production libconfig->ROS2 endpoint profile loader.
// Distinct robot files must not silently share global auxiliary velocity topics.
TEST(ConfigEndpointProfiles, AuxiliaryVelocityBindingsAreIndependent) {
  const auto path_a = fs::temp_directory_path() / "rc_agent_p2_aux_alpha.cfg";
  const auto path_b = fs::temp_directory_path() / "rc_agent_p2_aux_beta.cfg";
  {
    std::ofstream out(path_a);
    out << "Amur: { API: { endpoints: { "
           "odom_topic = \"/alpha/odom\"; cmd_vel_topic = \"/alpha/cmd_vel\"; "
           "cmd_vel_smoothed_topic = \"/alpha/filtered\"; "
           "cmd_vel_nav_topic = \"/alpha/nav\"; }; }; };\n";
  }
  {
    std::ofstream out(path_b);
    out << "Amur: { API: { endpoints: { "
           "odom_topic = \"/beta/odom\"; cmd_vel_topic = \"/beta/cmd_vel\"; "
           "cmd_vel_smoothed_topic = \"/beta/filtered\"; "
           "cmd_vel_nav_topic = \"/beta/nav\"; "
           "cmd_vel_type = \"twist_stamped\"; }; }; };\n";
  }
  ConfigProcessor a(path_a.string());
  ConfigProcessor b(path_b.string());
  const auto alpha = loadBaseControlEndpointProfile(a);
  const auto beta = loadBaseControlEndpointProfile(b);
  EXPECT_EQ(alpha.odom_topic, "/alpha/odom");
  EXPECT_EQ(beta.odom_topic, "/beta/odom");
  EXPECT_EQ(alpha.cmd_vel_topic, "/alpha/cmd_vel");
  EXPECT_EQ(beta.cmd_vel_topic, "/beta/cmd_vel");
  EXPECT_EQ(alpha.cmd_vel_smoothed_topic, "/alpha/filtered");
  EXPECT_EQ(beta.cmd_vel_smoothed_topic, "/beta/filtered");
  EXPECT_EQ(alpha.cmd_vel_nav_topic, "/alpha/nav");
  EXPECT_EQ(beta.cmd_vel_nav_topic, "/beta/nav");
  EXPECT_EQ(alpha.cmd_vel_type, "twist");
  EXPECT_EQ(beta.cmd_vel_type, "twist_stamped");
  EXPECT_EQ(alpha.cmd_vel_frame, "base_link");
  EXPECT_EQ(beta.cmd_vel_frame, "base_link");
  fs::remove(path_a);
  fs::remove(path_b);
}

TEST(ConfigEndpointProfiles, MissingAuxiliaryBindingsRetainLegacyDefaults) {
  const auto path = fs::temp_directory_path() / "rc_agent_p2_aux_defaults.cfg";
  {
    std::ofstream out(path);
    out << "Amur: { API: { endpoints: { cmd_vel_topic = \"/custom/cmd_vel\"; }; }; };\n";
  }
  ConfigProcessor config(path.string());
  const auto profile = loadBaseControlEndpointProfile(config);
  EXPECT_EQ(profile.cmd_vel_topic, "/custom/cmd_vel");
  EXPECT_EQ(profile.cmd_vel_smoothed_topic, "/cmd_vel_smoothed");
  EXPECT_EQ(profile.cmd_vel_nav_topic, "/cmd_vel_nav");
  fs::remove(path);
}

TEST(ConfigEndpointProfiles, InvalidAuxiliaryBindingsFailClosed) {
  const auto path = fs::temp_directory_path() / "rc_agent_p2_aux_invalid.cfg";
  for (const auto &key : {"cmd_vel_smoothed_topic", "cmd_vel_nav_topic"}) {
    for (const auto &bad_value : {"17", "\"\""}) {
      {
        std::ofstream out(path);
        out << "Amur: { API: { endpoints: { " << key
            << " = " << bad_value << "; }; }; };\n";
      }
      ConfigProcessor config(path.string());
      EXPECT_THROW(loadBaseControlEndpointProfile(config), std::invalid_argument)
          << "key=" << key << " value=" << bad_value;
    }
  }
  fs::remove(path);
}


// P2 production path: the same libconfig Nav2 resolver is used by main.cpp,
// not test-only hard-coded NodeOptions. Two robot profiles configure every
// supported action and localization endpoint without recompilation.
TEST(ConfigEndpointProfiles, Nav2ActionsLoadIndependently) {
  const auto path_a = fs::temp_directory_path() / "rc_agent_p2_nav_alpha.cfg";
  const auto path_b = fs::temp_directory_path() / "rc_agent_p2_nav_beta.cfg";
  {
    std::ofstream out(path_a);
    out << R"(Amur: { API: { endpoints: {
      navigate_to_pose_action = "/alpha/navigate_to_pose";
      navigate_through_poses_action = "/alpha/navigate_through_poses";
      follow_waypoints_action = "/alpha/follow_waypoints";
      spin_action = "/alpha/spin";
      initial_pose_topic = "/alpha/initialpose";
    }; }; };)";
  }
  {
    std::ofstream out(path_b);
    out << R"(Amur: { API: { endpoints: {
      navigate_to_pose_action = "/beta/navigate_to_pose";
      navigate_through_poses_action = "/beta/navigate_through_poses";
      follow_waypoints_action = "/beta/follow_waypoints";
      spin_action = "/beta/spin";
      initial_pose_topic = "/beta/initialpose";
    }; }; };)";
  }
  ConfigProcessor a(path_a.string());
  ConfigProcessor b(path_b.string());
  const auto alpha = loadNav2EndpointProfile(a);
  const auto beta = loadNav2EndpointProfile(b);
  EXPECT_EQ(alpha.navigate_to_pose_action, "/alpha/navigate_to_pose");
  EXPECT_EQ(alpha.navigate_through_poses_action, "/alpha/navigate_through_poses");
  EXPECT_EQ(alpha.follow_waypoints_action, "/alpha/follow_waypoints");
  EXPECT_EQ(alpha.spin_action, "/alpha/spin");
  EXPECT_EQ(alpha.initial_pose_topic, "/alpha/initialpose");
  EXPECT_EQ(beta.navigate_to_pose_action, "/beta/navigate_to_pose");
  EXPECT_EQ(beta.navigate_through_poses_action, "/beta/navigate_through_poses");
  EXPECT_EQ(beta.follow_waypoints_action, "/beta/follow_waypoints");
  EXPECT_EQ(beta.spin_action, "/beta/spin");
  EXPECT_EQ(beta.initial_pose_topic, "/beta/initialpose");
  fs::remove(path_a);
  fs::remove(path_b);
}

TEST(ConfigEndpointProfiles, Nav2MissingKeysRetainLegacyDefaults) {
  const auto path = fs::temp_directory_path() / "rc_agent_p2_nav_defaults.cfg";
  {
    std::ofstream out(path);
    out << R"(Amur: { API: { endpoints: {
      navigate_to_pose_action = "/custom/navigate_to_pose";
    }; }; };)";
  }
  ConfigProcessor config(path.string());
  const auto profile = loadNav2EndpointProfile(config);
  EXPECT_EQ(profile.navigate_to_pose_action, "/custom/navigate_to_pose");
  EXPECT_EQ(profile.navigate_through_poses_action, "navigate_through_poses");
  EXPECT_EQ(profile.follow_waypoints_action, "follow_waypoints");
  EXPECT_EQ(profile.spin_action, "spin");
  EXPECT_EQ(profile.initial_pose_topic, "initialpose");
  fs::remove(path);
}

TEST(ConfigEndpointProfiles, Nav2MalformedEndpointValuesFailClosed) {
  const auto path = fs::temp_directory_path() / "rc_agent_p2_nav_invalid.cfg";
  for (const auto &key : {"navigate_to_pose_action",
                          "navigate_through_poses_action",
                          "follow_waypoints_action", "spin_action",
                          "initial_pose_topic"}) {
    for (const auto &invalid : {"42", "\"\""}) {
      {
        std::ofstream out(path);
        out << "Amur: { API: { endpoints: { " << key
            << " = " << invalid << "; }; }; };";
      }
      ConfigProcessor config(path.string());
      EXPECT_THROW(loadNav2EndpointProfile(config), std::invalid_argument)
          << "key=" << key << " value=" << invalid;
    }
  }
  fs::remove(path);
}
