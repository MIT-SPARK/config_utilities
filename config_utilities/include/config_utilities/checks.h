#pragma once
#include <sstream>
#include <string>

namespace config {

struct CheckBase {
  virtual ~CheckBase() = default;
  inline operator bool() const { return valid(); }
  virtual bool valid() const = 0;
  virtual std::string message() const = 0;
};

class Check : public CheckBase {
 public:
  Check(bool valid, const std::string& message) : valid_(valid), message_(message) {}

  virtual ~Check() = default;

  inline virtual bool valid() const override { return valid_; }

  inline virtual std::string message() const override { return message_; }

 protected:
  bool valid_;
  std::string message_;
};

/** @brief Trait to determine warning message for binary comparison operator
 * Specialize for your type to change the warning message when a check fails.
 */
template <typename Compare>
struct CompareMessageTrait {
  static std::string message() { return "did not pass comparison to"; };
};

template <typename T, typename Compare>
class BinaryCheck : public CheckBase {
 public:
  BinaryCheck(const T& param, const T& value, const std::string name = "")
      : param_(param), value_(value), name_(name) {}

  virtual bool valid() const override { return Compare{}(param_, value_); }

  virtual std::string message() const override {
    std::stringstream ss;
    ss << "Check " << name_ << " failed. Param value " << param_ << " " << CompareMessageTrait<Compare>::message()
       << " " << value_;
    return ss.str();
  }

 protected:
  T param_;
  T value_;
  std::string name_;
};

template <typename T>
struct CompareMessageTrait<std::greater<T>> {
  static std::string message() { return "is not greater than"; };
};

template <typename T>
struct CompareMessageTrait<std::greater_equal<T>> {
  static std::string message() { return "is not greater than or equal to"; };
};

template <typename T>
struct CompareMessageTrait<std::less<T>> {
  static std::string message() { return "is not less than"; };
};

template <typename T>
struct CompareMessageTrait<std::less_equal<T>> {
  static std::string message() { return "is not less than or equal to"; };
};

template <typename T>
struct CompareMessageTrait<std::equal_to<T>> {
  static std::string message() { return "is not equal to"; };
};

template <typename T>
struct CompareMessageTrait<std::not_equal_to<T>> {
  static std::string message() { return "is equal to"; };
};

template <typename T>
class CheckRange : public CheckBase {
 public:
  CheckRange(const T& param,
             const T& lower,
             const T& upper,
             const std::string& name,
             bool lower_inclusive = true,
             bool upper_inclusive = true)
      : param_(param),
        lower_(lower),
        upper_(upper),
        name_(name),
        lower_inclusive_(lower_inclusive),
        upper_inclusive_(upper_inclusive) {}

  virtual bool valid() const override {
    bool lower_okay = lower_inclusive_ ? param_ >= lower_ : param_ > lower_;
    bool upper_okay = upper_inclusive_ ? param_ <= upper_ : param_ < upper_;
    return lower_okay && upper_okay;
  }

  virtual std::string message() const override {
    std::stringstream ss;
    ss << "Param '" << name_ << "' is expected to be within " << (lower_inclusive_ ? "[" : "(") << lower_ << ", "
       << upper_ << (upper_inclusive_ ? "]" : ")") << " (is: '" << param_ << "').";
    return ss.str();
  }

 protected:
  T param_;
  T lower_;
  T upper_;
  std::string name_;
  bool lower_inclusive_;
  bool upper_inclusive_;
};

}  // namespace config
