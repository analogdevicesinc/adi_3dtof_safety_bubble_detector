/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#ifndef ADI_3DTOF_SAFETY_BUBBLE_DETECTOR_ZONE_CONFIG_HPP
#define ADI_3DTOF_SAFETY_BUBBLE_DETECTOR_ZONE_CONFIG_HPP

#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace adi_3dtof_safety_bubble_detector
{

/**
 * @brief Enumeration for zone shapes
 */
enum class ZoneShape
{
  CIRCLE = 0,
  RECTANGLE = 1
};

/**
 * @brief Convert ZoneShape enum to string
 */
inline std::string zoneShapeToString(ZoneShape shape)
{
  switch (shape) {
    case ZoneShape::CIRCLE:
      return "circle";
    case ZoneShape::RECTANGLE:
      return "rectangle";
    default:
      return "unknown";
  }
}

/**
 * @brief Convert string to ZoneShape enum
 */
inline ZoneShape stringToZoneShape(const std::string & shape_str)
{
  if (shape_str == "circle" || shape_str == "CIRCLE") {
    return ZoneShape::CIRCLE;
  } else if (shape_str == "rectangle" || shape_str == "RECTANGLE") {
    return ZoneShape::RECTANGLE;
  }
  return ZoneShape::CIRCLE;  // Default to circle
}

/**
 * @brief Structure to hold configuration for a single zone
 */
struct ZoneConfig
{
  int id = 1;                                              ///< Zone identifier (1 = innermost)
  bool enabled = true;                                     ///< Whether zone is active
  ZoneShape shape = ZoneShape::CIRCLE;                     ///< Shape of the zone
  float radius_mtr = 1.0f;                                 ///< Radius for circular zone (meters)
  float x_dim_mtr = 1.0f;                                  ///< Half-width (X) for rectangular zone - distance from center (meters)
  float z_dim_mtr = 1.0f;                                  ///< Half-height (Z) for rectangular zone - distance from center (meters)
  std::array<float, 4> color_rgba = {1.0f, 0.0f, 0.0f, 0.4f};  ///< Visualization color

  /**
   * @brief Get the effective radius in meters for calculations
   */
  float getRadiusMeters() const { return radius_mtr; }

  /**
   * @brief Get the X half-dimension in meters (distance from center)
   */
  float getXDimMeters() const { return x_dim_mtr; }

  /**
   * @brief Get the Z half-dimension in meters (distance from center)
   */
  float getZDimMeters() const { return z_dim_mtr; }

  /**
   * @brief Validate zone configuration
   * @return true if configuration is valid
   */
  bool isValid() const
  {
    if (id < 1) return false;
    if (shape == ZoneShape::CIRCLE && radius_mtr <= 0.0f) return false;
    if (shape == ZoneShape::RECTANGLE && (x_dim_mtr <= 0.0f || z_dim_mtr <= 0.0f)) return false;
    return true;
  }

  /**
   * @brief Get the maximum extent of the zone (for comparison purposes)
   * @return Maximum radius or half-diagonal in meters
   */
  float getMaxExtent() const
  {
    if (shape == ZoneShape::CIRCLE) {
      return radius_mtr;
    } else {
      // Half diagonal for rectangle (x_dim_mtr and z_dim_mtr are already half-dimensions)
      return std::sqrt(x_dim_mtr * x_dim_mtr + z_dim_mtr * z_dim_mtr);
    }
  }
};

/**
 * @brief Get default color for a zone based on its index
 *        Zone 1 = Red, Zone 2 = Yellow, Zone 3+ = Green gradient
 * @param zone_id Zone identifier (1-based)
 * @param num_zones Total number of zones
 * @param detected Whether object is detected (increases alpha)
 * @return RGBA color array
 */
inline std::array<float, 4> getZoneColor(int zone_id, int num_zones, bool detected = false)
{
  std::array<float, 4> color;
  float base_alpha = detected ? 0.6f : 0.3f;

  if (zone_id == 1) {
    // Zone 1: Red
    color = {1.0f, 0.0f, 0.0f, base_alpha + 0.1f};
  } else if (zone_id == 2) {
    // Zone 2: Yellow
    color = {1.0f, 1.0f, 0.0f, base_alpha};
  } else {
    // Zone 3+: Interpolate from yellow to green
    float t = 0.0f;
    if (num_zones > 2) {
      t = static_cast<float>(zone_id - 2) / static_cast<float>(num_zones - 2);
    }
    // Yellow (1,1,0) to Green (0,1,0)
    color = {1.0f - t, 1.0f, 0.0f, base_alpha - 0.1f};
  }

  return color;
}

/**
 * @brief Class to manage multiple safety zones
 */
class MultiZoneConfig
{
public:
  MultiZoneConfig() : num_zones_(3) { initializeDefaultZones(); }

  explicit MultiZoneConfig(int num_zones) : num_zones_(num_zones) { initializeDefaultZones(); }

  /**
   * @brief Initialize default zone configurations
   */
  void initializeDefaultZones()
  {
    zones_.clear();
    zones_.reserve(num_zones_);

    for (int i = 1; i <= num_zones_; ++i) {
      ZoneConfig zone;
      zone.id = i;
      zone.enabled = true;
      zone.shape = ZoneShape::CIRCLE;
      // Default radii: Zone 1 = 0.5m, Zone 2 = 1.0m, Zone 3 = 1.5m, etc.
      zone.radius_mtr = 0.5f * static_cast<float>(i);
      zone.x_dim_mtr = zone.radius_mtr;
      zone.z_dim_mtr = zone.radius_mtr;
      zone.color_rgba = getZoneColor(i, num_zones_);
      zones_.push_back(zone);
    }
  }

  /**
   * @brief Set the number of zones and reinitialize
   */
  void setNumZones(int num_zones)
  {
    if (num_zones < 1) num_zones = 1;
    num_zones_ = num_zones;
    initializeDefaultZones();
  }

  /**
   * @brief Get the number of zones
   */
  int getNumZones() const { return num_zones_; }

  /**
   * @brief Get all zone configurations
   */
  std::vector<ZoneConfig> & getZones() { return zones_; }

  /**
   * @brief Get all zone configurations (const)
   */
  const std::vector<ZoneConfig> & getZones() const { return zones_; }

  /**
   * @brief Get a specific zone configuration
   * @param zone_id Zone identifier (1-based)
   * @return Pointer to zone config or nullptr if not found
   */
  ZoneConfig * getZone(int zone_id)
  {
    for (auto & zone : zones_) {
      if (zone.id == zone_id) {
        return &zone;
      }
    }
    return nullptr;
  }

  /**
   * @brief Get a specific zone configuration (const)
   */
  const ZoneConfig * getZone(int zone_id) const
  {
    for (const auto & zone : zones_) {
      if (zone.id == zone_id) {
        return &zone;
      }
    }
    return nullptr;
  }

  /**
   * @brief Update a zone's configuration
   * @param zone_id Zone identifier
   * @param config New configuration
   * @return true if zone was found and updated
   */
  bool updateZone(int zone_id, const ZoneConfig & config)
  {
    for (auto & zone : zones_) {
      if (zone.id == zone_id) {
        zone = config;
        zone.id = zone_id;  // Preserve original ID
        zone.color_rgba = getZoneColor(zone_id, num_zones_);
        return true;
      }
    }
    return false;
  }

  /**
   * @brief Add a new zone
   */
  void addZone()
  {
    num_zones_++;
    ZoneConfig zone;
    zone.id = num_zones_;
    zone.enabled = true;
    zone.shape = ZoneShape::CIRCLE;
    zone.radius_mtr = 0.5f * static_cast<float>(num_zones_);
    zone.x_dim_mtr = zone.radius_mtr;
    zone.z_dim_mtr = zone.radius_mtr;
    zone.color_rgba = getZoneColor(num_zones_, num_zones_);
    zones_.push_back(zone);

    // Update colors for all zones
    updateAllColors();
  }

  /**
   * @brief Remove the outermost zone
   * @return true if a zone was removed
   */
  bool removeZone()
  {
    if (num_zones_ <= 1) return false;

    zones_.pop_back();
    num_zones_--;
    updateAllColors();
    return true;
  }

  /**
   * @brief Update colors for all zones based on current count
   */
  void updateAllColors()
  {
    for (auto & zone : zones_) {
      zone.color_rgba = getZoneColor(zone.id, num_zones_);
    }
  }

  /**
   * @brief Validate the entire configuration
   * @return Pair of (valid, error_message)
   */
  std::pair<bool, std::string> validate() const
  {
    if (zones_.empty()) {
      return {false, "No zones configured"};
    }

    for (const auto & zone : zones_) {
      if (!zone.isValid()) {
        return {false, "Zone " + std::to_string(zone.id) + " has invalid configuration"};
      }
    }

    // Note: Zones do NOT need to be nested or ordered by size
    // Overlapping and independent zones are allowed by design

    return {true, "Configuration valid"};
  }

  /**
   * @brief Load configuration from YAML file
   * @param filepath Path to YAML file
   * @return true if loaded successfully
   */
  bool loadFromYaml(const std::string & filepath)
  {
    std::ifstream file(filepath);
    if (!file.is_open()) {
      std::cerr << "Failed to open YAML file: " << filepath << std::endl;
      return false;
    }

    zones_.clear();
    std::string line;
    int current_zone_id = -1;
    ZoneConfig current_zone;
    bool in_zones_section = false;

    while (std::getline(file, line)) {
      // Trim whitespace
      size_t start = line.find_first_not_of(" \t");
      if (start == std::string::npos) continue;
      line = line.substr(start);

      // Skip comments
      if (line[0] == '#') continue;

      // Parse num_zones
      if (line.find("num_zones:") != std::string::npos) {
        size_t pos = line.find(':');
        if (pos != std::string::npos) {
          num_zones_ = std::stoi(line.substr(pos + 1));
        }
      }
      // Check for zones section
      else if (line.find("zones:") != std::string::npos) {
        in_zones_section = true;
      }
      // Parse zone entries
      else if (in_zones_section && line.find("- id:") != std::string::npos) {
        // Save previous zone if exists
        if (current_zone_id > 0) {
          current_zone.color_rgba = getZoneColor(current_zone.id, num_zones_);
          zones_.push_back(current_zone);
        }
        // Start new zone
        current_zone = ZoneConfig();
        size_t pos = line.find(':');
        if (pos != std::string::npos) {
          current_zone_id = std::stoi(line.substr(pos + 1));
          current_zone.id = current_zone_id;
        }
      } else if (in_zones_section && current_zone_id > 0) {
        // Parse zone properties
        if (line.find("enabled:") != std::string::npos) {
          current_zone.enabled = (line.find("true") != std::string::npos);
        } else if (line.find("shape:") != std::string::npos) {
          size_t pos = line.find(':');
          if (pos != std::string::npos) {
            std::string shape_str = line.substr(pos + 1);
            // Remove quotes and whitespace
            size_t quote_start = shape_str.find('"');
            size_t quote_end = shape_str.rfind('"');
            if (quote_start != std::string::npos && quote_end != std::string::npos) {
              shape_str = shape_str.substr(quote_start + 1, quote_end - quote_start - 1);
            }
            // Trim
            size_t s = shape_str.find_first_not_of(" \t");
            size_t e = shape_str.find_last_not_of(" \t");
            if (s != std::string::npos) {
              shape_str = shape_str.substr(s, e - s + 1);
            }
            current_zone.shape = stringToZoneShape(shape_str);
          }
        } else if (line.find("radius_mtr:") != std::string::npos) {
          size_t pos = line.find(':');
          if (pos != std::string::npos) {
            current_zone.radius_mtr = std::stof(line.substr(pos + 1));
          }
        } else if (line.find("x_dim_mtr:") != std::string::npos) {
          size_t pos = line.find(':');
          if (pos != std::string::npos) {
            current_zone.x_dim_mtr = std::stof(line.substr(pos + 1));
          }
        } else if (line.find("z_dim_mtr:") != std::string::npos) {
          size_t pos = line.find(':');
          if (pos != std::string::npos) {
            current_zone.z_dim_mtr = std::stof(line.substr(pos + 1));
          }
        }
      }
    }

    // Save last zone
    if (current_zone_id > 0) {
      current_zone.color_rgba = getZoneColor(current_zone.id, num_zones_);
      zones_.push_back(current_zone);
    }

    file.close();
    return !zones_.empty();
  }

  /**
   * @brief Save configuration to YAML file
   * @param filepath Path to YAML file
   * @return true if saved successfully
   */
  bool saveToYaml(const std::string & filepath) const
  {
    std::ofstream file(filepath);
    if (!file.is_open()) {
      std::cerr << "Failed to open YAML file for writing: " << filepath << std::endl;
      return false;
    }

    file << "# Safety Bubble Detector - Multi-Zone Configuration\n";
    file << "# Generated configuration file\n\n";
    file << "safety_zones:\n";
    file << "  num_zones: " << num_zones_ << "\n";
    file << "  zones:\n";

    for (const auto & zone : zones_) {
      file << "    - id: " << zone.id << "\n";
      file << "      enabled: " << (zone.enabled ? "true" : "false") << "\n";
      file << "      shape: \"" << zoneShapeToString(zone.shape) << "\"\n";
      file << "      radius_mtr: " << zone.radius_mtr << "\n";
      file << "      x_dim_mtr: " << zone.x_dim_mtr << "\n";
      file << "      z_dim_mtr: " << zone.z_dim_mtr << "\n";
    }

    file.close();
    return true;
  }

private:
  int num_zones_;
  std::vector<ZoneConfig> zones_;
};

}  // namespace adi_3dtof_safety_bubble_detector

#endif  // ADI_3DTOF_SAFETY_BUBBLE_DETECTOR_ZONE_CONFIG_HPP
