#include "config_utilities/config.h"
#include "config_utilities/factory.h"

namespace config::test {

struct Talker {
  virtual ~Talker() = default;
  virtual std::string talk() const = 0;
};

struct InternalTalker : Talker {
  std::string talk() const override { return "internal"; }
  inline static const auto reg = config::Registration<Talker, InternalTalker>("internal");
};

struct RepeatingTalker : Talker {
  struct Config {
    size_t repeats = 5;
    std::string message = "hello";
  } const config;

  explicit RepeatingTalker(const Config& config) : config(config) {}

  std::string talk() const override {
    std::stringstream ss;
    for (size_t i = 0; i < config.repeats; ++i) {
      ss << config.message;
    }
    return ss.str();
  }

  inline static const auto reg = config::RegistrationWithConfig<Talker, RepeatingTalker, Config>("repeating");
};

void declare_config(RepeatingTalker::Config& config) {
  config::name("RepeatingTalker::Config");
  config::field(config.repeats, "repeats");
  config::field(config.message, "message");
}

}  // namespace config::test
