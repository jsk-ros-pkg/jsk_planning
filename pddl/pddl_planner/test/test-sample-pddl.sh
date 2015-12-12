#!/bin/bash

echo $@
for OPT in "$@"; do
    case "$OPT" in
        __name:=* )
            NAME=`echo $1 | sed 's/__name:=//g'`
            shift 1
            ;;
        --gtest_output=xml:* )
            OUTPUT=`echo $1 | sed 's/--gtest_output=xml://g'`
            shift 1
            ;;
        _* )
            shift 1
            ;;
        --text )
            shift 1
            ;;
        *)
            CMD+="$OPT "
            shift 1
            ;;
    esac
done

echo "======================"
echo $CMD
#$CMD > $OUTPUT
$CMD > $OUTPUT.tmp
ret=$?
if [ "$ret" == 0 ]; then
cat <<EOF > $OUTPUT
<?xml version="1.0" encoding="utf-8"?>
<testsuite errors="0" failures="0" name="unittest.suite.TestSuite" tests="1" time="1">
  <testcase classname="rostest.runner.RosTest" name="$NAME" time="1">
  </testcase>
  <system-out><![CDATA[
$(cat $OUTPUT.tmp)
]]></system-out>
</testsuite>

EOF
else
cat <<EOF > $OUTPUT
<?xml version="1.0" encoding="utf-8"?>
<testsuite errors="1" failures="0" name="unittest.suite.TestSuite" tests="1" time="1">
  <testcase classname="rostest.runner.RosTest" name="$NAME" time="1">
    <failure type="AssertionError"/>
  </testcase>
  <system-err><![CDATA[
$(cat $OUTPUT.tmp)
]]></system-err>
</testsuite>

EOF
fi
