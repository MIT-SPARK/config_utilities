#include "config_utilities/internal/namespacing.h"

#include <gtest/gtest.h>

#include "config_utilities/config.h"
#include "config_utilities/internal/string_utils.h"
#include "config_utilities/test/utils.h"

namespace config::test {

bool kLeaveNameSpaceOpen = true;

struct Ns {
  virtual ~Ns() = default;
  std::vector<std::string> ns = std::vector<std::string>(18, "");
};

void declare_config(Ns& ns) {
  // Tests using the namespace functions syntax.
  ns.ns[0] = config::current_namespace();

  config::enter_namespace("sub_ns");
  ns.ns[1] = config::current_namespace();

  config::enter_namespace("sub_sub_ns");
  ns.ns[2] = config::current_namespace();

  config::exit_namespace();
  ns.ns[3] = config::current_namespace();

  config::enter_namespace("different_ns");
  ns.ns[4] = config::current_namespace();

  config::switch_namespace("another_ns");
  ns.ns[5] = config::current_namespace();

  config::clear_namespaces();
  ns.ns[6] = config::current_namespace();

  // Tests using the NameSpace class syntax.
  {
    NameSpace open_ns("a");
    ns.ns[7] = config::current_namespace();
    {
      NameSpace open_ns2("b");
      ns.ns[8] = config::current_namespace();
    }
    ns.ns[9] = config::current_namespace();
  }
  ns.ns[10] = config::current_namespace();

  // Tests using the NameSpace class member functions.
  NameSpace first_ns("x");
  ns.ns[11] = config::current_namespace();

  NameSpace second_ns("y");
  ns.ns[12] = config::current_namespace();

  second_ns.exit();
  ns.ns[13] = config::current_namespace();

  NameSpace third_ns("z");
  second_ns.enter();
  ns.ns[14] = config::current_namespace();

  second_ns.exit();
  ns.ns[15] = config::current_namespace();

  // NOTE(lschmid): These are unintended use cases but should function properly and warn the user.
  second_ns.enter();
  third_ns.exit();
  ns.ns[16] = config::current_namespace();

  second_ns.exit();
  ns.ns[17] = config::current_namespace();

  first_ns.exit();
  if (kLeaveNameSpaceOpen) {
    // Intentionally leave a namespace open to test the cleanup.
    enter_namespace("leftover_ns");
  } else {
    // Intentionally clear all namespaces to test the cleanup.
    clear_namespaces();
  }
}

struct DerivedNs : public Ns {
  enum class Syntax { kNone, kNamespaceFunctions, kNameSpaceClass } const syntax = Syntax::kNone;
  explicit DerivedNs(Syntax syntax) : syntax(syntax) {}

  Ns ns;
};

void declare_config(DerivedNs& ns) {
  switch (ns.syntax) {
    case DerivedNs::Syntax::kNone: {
      config::base<Ns>(ns);
      config::field(ns.ns, "ns");
      break;
    }
    case DerivedNs::Syntax::kNamespaceFunctions: {
      config::enter_namespace("function_ns");
      config::base<Ns>(ns);
      config::exit_namespace();
      config::field(ns.ns, "ns");
      break;
    }
    case DerivedNs::Syntax::kNameSpaceClass: {
      {
        NameSpace open_ns("class_ns");
        config::base<Ns>(ns);
      }
      config::field(ns.ns, "ns");

      break;
    }
  }
}

struct DoublyDerivedNs : public DerivedNs {
  explicit DoublyDerivedNs(Syntax syntax) : DerivedNs(syntax) {}
};

void declare_config(DoublyDerivedNs& ns) {
  switch (ns.syntax) {
    case DerivedNs::Syntax::kNone: {
      config::base<DerivedNs>(ns);
      break;
    }
    case DerivedNs::Syntax::kNamespaceFunctions: {
      config::enter_namespace("double_function_ns");
      config::base<DerivedNs>(ns);
      break;
    }
    case DerivedNs::Syntax::kNameSpaceClass: {
      {
        NameSpace open_ns("double_class_ns");
        config::base<DerivedNs>(ns);
      }
      break;
    }
  }
}

struct NsWrapper {
  Ns ns1, ns2;
};

void declare_config(NsWrapper& ns) {
  config::field(ns.ns1, "ns1");
  NameSpace open_ns("a/b/");
  config::field(ns.ns2, "ns2");
}

void checkNS(const Ns& ns, const std::string& ns_prefix = "") {
  EXPECT_EQ(ns.ns[0], internal::joinNamespace(ns_prefix, ""));
  EXPECT_EQ(ns.ns[1], internal::joinNamespace(ns_prefix, "sub_ns"));
  EXPECT_EQ(ns.ns[2], internal::joinNamespace(ns_prefix, "sub_ns/sub_sub_ns"));
  EXPECT_EQ(ns.ns[3], internal::joinNamespace(ns_prefix, "sub_ns"));
  EXPECT_EQ(ns.ns[4], internal::joinNamespace(ns_prefix, "sub_ns/different_ns"));
  EXPECT_EQ(ns.ns[5], internal::joinNamespace(ns_prefix, "sub_ns/another_ns"));
  EXPECT_EQ(ns.ns[6], internal::joinNamespace(ns_prefix, ""));
  EXPECT_EQ(ns.ns[7], internal::joinNamespace(ns_prefix, "a"));
  EXPECT_EQ(ns.ns[8], internal::joinNamespace(ns_prefix, "a/b"));
  EXPECT_EQ(ns.ns[9], internal::joinNamespace(ns_prefix, "a"));
  EXPECT_EQ(ns.ns[10], internal::joinNamespace(ns_prefix, ""));
  EXPECT_EQ(ns.ns[11], internal::joinNamespace(ns_prefix, "x"));
  EXPECT_EQ(ns.ns[12], internal::joinNamespace(ns_prefix, "x/y"));
  EXPECT_EQ(ns.ns[13], internal::joinNamespace(ns_prefix, "x"));
  EXPECT_EQ(ns.ns[14], internal::joinNamespace(ns_prefix, "x/z/y"));
  EXPECT_EQ(ns.ns[15], internal::joinNamespace(ns_prefix, "x/z"));
  EXPECT_EQ(ns.ns[16], internal::joinNamespace(ns_prefix, "x"));
  EXPECT_EQ(ns.ns[17], internal::joinNamespace(ns_prefix, "x"));
}

TEST(Namespacing, splitNamespace) {
  std::string ns = "a/b/c";
  const std::vector<std::string> ns_expected = {"a", "b", "c"};
  EXPECT_EQ(internal::splitNamespace(ns), ns_expected);

  ns = "/a/b/c/";
  EXPECT_EQ(internal::splitNamespace(ns), ns_expected);

  ns = "a/b///c/";
  EXPECT_EQ(internal::splitNamespace(ns), ns_expected);
}

TEST(Namespacing, joinNamespace) {
  std::vector<std::string> ns = {"a", "b", "c"};
  const std::string ns_expected = "a/b/c";
  EXPECT_EQ(internal::joinNamespace(ns), ns_expected);

  std::string ns1 = "a";
  std::string ns2 = "b/c";
  EXPECT_EQ(internal::joinNamespace(ns1, ns2), ns_expected);

  ns1 = "/a";
  ns2 = "/b/c";
  EXPECT_EQ(internal::joinNamespace(ns1, ns2), ns_expected);

  ns1 = "a/b///";
  ns2 = "//c/";
  EXPECT_EQ(internal::joinNamespace(ns1, ns2), ns_expected);

  ns1 = "";
  ns2 = "";
  EXPECT_EQ(internal::joinNamespace(ns1, ns2), "");

  ns1 = "a/b/c";
  ns2 = "";
  EXPECT_EQ(internal::joinNamespace(ns1, ns2), ns_expected);

  ns1 = "";
  ns2 = "a/b/c";
  EXPECT_EQ(internal::joinNamespace(ns1, ns2), ns_expected);
}

TEST(Namespacing, enterNamespace) {
  auto logger = std::make_shared<TestLogger>();
  internal::Logger::setLogger(logger);
  Ns ns;
  internal::Visitor::setValues(ns, YAML::Node());
  checkNS(ns);

  // This should warn about improper use of NameSpace.enter()/exit().
  EXPECT_EQ(logger->messages().size(), 2);
}

TEST(Namespacing, enterNestedNamespaces) {
  auto testNestedNs = []() {
    auto derived_ns_none = DerivedNs(DerivedNs::Syntax::kNone);
    internal::Visitor::setValues(derived_ns_none, YAML::Node());
    checkNS(derived_ns_none);
    checkNS(derived_ns_none.ns);

    auto derived_ns_class = DerivedNs(DerivedNs::Syntax::kNameSpaceClass);
    internal::Visitor::setValues(derived_ns_class, YAML::Node());
    checkNS(derived_ns_class, "class_ns");
    checkNS(derived_ns_class.ns);

    auto derived_ns_fn = DerivedNs(DerivedNs::Syntax::kNamespaceFunctions);
    internal::Visitor::setValues(derived_ns_fn, YAML::Node());
    checkNS(derived_ns_fn, "function_ns");
    checkNS(derived_ns_fn.ns);

    auto doubly_derived_ns_none = DoublyDerivedNs(DerivedNs::Syntax::kNone);
    internal::Visitor::setValues(doubly_derived_ns_none, YAML::Node());
    checkNS(doubly_derived_ns_none);
    checkNS(doubly_derived_ns_none.ns);

    auto doubly_derived_ns_class = DoublyDerivedNs(DerivedNs::Syntax::kNameSpaceClass);
    internal::Visitor::setValues(doubly_derived_ns_class, YAML::Node());
    checkNS(doubly_derived_ns_class, "double_class_ns/class_ns");
    checkNS(doubly_derived_ns_class.ns, "double_class_ns");

    auto doubly_derived_ns_fn = DoublyDerivedNs(DerivedNs::Syntax::kNamespaceFunctions);
    internal::Visitor::setValues(doubly_derived_ns_fn, YAML::Node());
    checkNS(doubly_derived_ns_fn, "double_function_ns/function_ns");
    checkNS(doubly_derived_ns_fn.ns, "double_function_ns");

    NsWrapper ns_wrapper;
    internal::Visitor::setValues(ns_wrapper, YAML::Node());
    checkNS(ns_wrapper.ns1, "");
    checkNS(ns_wrapper.ns2, "a/b");
  };

  kLeaveNameSpaceOpen = true;
  testNestedNs();

  kLeaveNameSpaceOpen = false;
  testNestedNs();
}

}  // namespace config::test
