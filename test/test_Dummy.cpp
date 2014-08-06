#include <boost/test/unit_test.hpp>
#include <dvl_rti/Dummy.hpp>

using namespace dvl_rti;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    dvl_rti::DummyClass dummy;
    dummy.welcome();
}
