/** -----------------------------------------------------------------------------
 * Copyright (c) 2023 Massachusetts Institute of Technology.
 * All Rights Reserved.
 *
 * AUTHORS:     Lukas Schmid <lschmid@mit.edu>, Nathan Hughes <na26933@mit.edu>
 * AFFILIATION: MIT-SPARK Lab, Massachusetts Institute of Technology
 * YEAR:        2023
 * LICENSE:     BSD 3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * -------------------------------------------------------------------------- */

#pragma once

#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace config::internal {

struct CheckBase {
  virtual ~CheckBase() = default;

  virtual bool valid() const = 0;
  virtual std::string message() const = 0;
  virtual std::string name() const { return ""; }
  virtual std::unique_ptr<CheckBase> clone() const = 0;

  inline operator bool() const { return valid(); }
};

class Check : public CheckBase {
 public:
  Check(bool valid, const std::string& message) : valid_(valid), message_(message) {}

  virtual ~Check() = default;

  inline bool valid() const override { return valid_; }

  inline std::string message() const override { return message_; }

  inline std::unique_ptr<CheckBase> clone() const override { return std::make_unique<Check>(valid_, message_); }

 protected:
  const bool valid_;
  const std::string message_;
};

/** @brief Trait to determine warning message for binary comparison operator
 * Specialize for your type to change the warning message when a check fails.
 */
template <typename Compare>
struct CompareMessageTrait {
  static std::string message() { return "compared to"; }
};

template <typename T, typename Compare>
class BinaryCheck : public CheckBase {
 public:
  BinaryCheck(const T& param, const T& value, const std::string name = "")
      : param_(param), value_(value), name_(name) {}

  bool valid() const override { return Compare{}(param_, value_); }

  std::string message() const override {
    std::stringstream ss;
    ss << "param " << CompareMessageTrait<Compare>::message() << " " << value_ << " (is: '" << param_ << "')";
    return ss.str();
  }

  std::string name() const override { return name_; }

  std::unique_ptr<CheckBase> clone() const override {
    return std::make_unique<BinaryCheck<T, Compare>>(param_, value_, name_);
  }

 protected:
  T param_;
  T value_;
  std::string name_;
};

template <typename T>
struct CompareMessageTrait<std::greater<T>> {
  static std::string message() { return ">"; }
};

template <typename T>
struct CompareMessageTrait<std::greater_equal<T>> {
  static std::string message() { return ">="; }
};

template <typename T>
struct CompareMessageTrait<std::less<T>> {
  static std::string message() { return "<"; }
};

template <typename T>
struct CompareMessageTrait<std::less_equal<T>> {
  static std::string message() { return "<="; }
};

template <typename T>
struct CompareMessageTrait<std::equal_to<T>> {
  static std::string message() { return "=="; }
};

template <typename T>
struct CompareMessageTrait<std::not_equal_to<T>> {
  static std::string message() { return "!="; }
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

  bool valid() const override {
    bool lower_okay = lower_inclusive_ ? param_ >= lower_ : param_ > lower_;
    bool upper_okay = upper_inclusive_ ? param_ <= upper_ : param_ < upper_;
    return lower_okay && upper_okay;
  }

  std::string message() const override {
    std::stringstream ss;
    ss << "param within " << (lower_inclusive_ ? "[" : "(") << lower_ << ", " << upper_
       << (upper_inclusive_ ? "]" : ")") << " (is: '" << param_ << "')";
    return ss.str();
  }

  std::string name() const override { return name_; }

  std::unique_ptr<CheckBase> clone() const override {
    return std::make_unique<CheckRange<T>>(param_, lower_, upper_, name_, lower_inclusive_, upper_inclusive_);
  }

 protected:
  const T param_;
  const T lower_;
  const T upper_;
  const std::string name_;
  const bool lower_inclusive_;
  const bool upper_inclusive_;
};

template <typename T>
class CheckIsOneOf : public CheckBase {
 public:
  CheckIsOneOf(const T& param, const std::vector<T>& candidates, const std::string& name)
      : param_(param), candidates_(candidates), name_(name) {}

  bool valid() const override {
    for (const T& cadidate : candidates_) {
      if (param_ == cadidate) {
        return true;
      }
    }
    return false;
  }

  std::string message() const override {
    std::stringstream candidates;
    bool is_first = true;
    for (const T& candidate : candidates_) {
      if (!is_first) {
        candidates << ", ";
      }
      is_first = false;
      candidates << "'" << candidate << "'";
    }
    std::stringstream ss;
    ss << "param must be one of [" << candidates.str() << "] (is: '" << param_ << "')";
    return ss.str();
  }

  std::string name() const override { return name_; }

  std::unique_ptr<CheckBase> clone() const override {
    return std::make_unique<CheckIsOneOf<T>>(param_, candidates_, name_);
  }

 private:
  const T param_;
  const std::vector<T> candidates_;
  const std::string name_;
};

}  // namespace config::internal
