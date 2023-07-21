#pragma once

#include <string>

namespace config::test {

std::string getResourcePath();

// template <typename T>
// void expectVectorEq(const std::vector<T>& a, const std::vector<T>& b) {
//   if (a.size() != b.size()) {
//     FAIL() << "Vectors differ in size: " << a.size() << " vs " << b.size();
//     return;
//   }
//   for (size_t i = 0; i < a.size(); ++i) {
//     if (a[i] != b[i]) {
//       FAIL() << "Vectors differ at index " << i;
//       return;
//     }
//   }
// }

}  // namespace config::test
