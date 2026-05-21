#!/bin/bash
# 模拟器航向角系统测试流程 - 完整自动化脚本

set -e

echo "╔════════════════════════════════════════════════════════════════════╗"
echo "║  SIMULATOR HEADING ANGLE CONTROL - AUTOMATED TEST SUITE            ║"
echo "╚════════════════════════════════════════════════════════════════════╝"
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 配置
UNITY_IP="${XEL_UNITY_IP:-127.0.0.1}"
TEST_OUTPUT_DIR="Records/heading_tests"
REAL_DATA_FILE="Software/Records/friction/friction_real_sampling_20260519_190045.csv"

echo -e "${BLUE}⚙️  Configuration:${NC}"
echo "   Unity IP: $UNITY_IP"
echo "   Output directory: $TEST_OUTPUT_DIR"
echo "   Real data: $REAL_DATA_FILE"
echo ""

# 检查文件
echo -e "${BLUE}📋 Pre-check:${NC}"

if [ ! -f "$REAL_DATA_FILE" ]; then
    echo -e "${RED}❌ Real robot data not found: $REAL_DATA_FILE${NC}"
    exit 1
fi
echo -e "${GREEN}✅ Real robot data found${NC}"

if [ ! -f "src/tests/test_sim_heading_fix.py" ]; then
    echo -e "${RED}❌ Test script not found: src/tests/test_sim_heading_fix.py${NC}"
    exit 1
fi
echo -e "${GREEN}✅ Test scripts found${NC}"

# 创建输出目录
mkdir -p "$TEST_OUTPUT_DIR"
echo -e "${GREEN}✅ Output directory ready${NC}"
echo ""

# Step 1: 诊断脚本（快速）
echo -e "${BLUE}📊 STEP 1: Running diagnostic analysis...${NC}"
python -m src.tests.diagnose_heading_issues 2>&1 | tee "$TEST_OUTPUT_DIR/diagnosis.log"
echo ""

# Step 2: 模拟器测试
echo -e "${BLUE}🤖 STEP 2: Testing simulator heading response...${NC}"
echo "   (This will take ~1-2 minutes depending on your test parameters)"
echo ""

TEST_CMD="XEL_UNITY_IP=$UNITY_IP \
TEST_WZ_CMD_LEVELS=0.1,0.2,0.5,1.0 \
TEST_HOLD_TIME=1.0 \
TEST_COAST_TIME=0.5 \
TEST_REPEAT=2 \
TEST_SAMPLE_HZ=20 \
python -m src.tests.test_sim_heading_fix"

eval "$TEST_CMD" 2>&1 | tee "$TEST_OUTPUT_DIR/test.log"

# 获取最新的测试输出文件
LATEST_JSON=$(ls -t "$TEST_OUTPUT_DIR"/heading_test_*.json 2>/dev/null | head -1)
if [ -z "$LATEST_JSON" ]; then
    echo -e "${YELLOW}⚠️  No test JSON file generated. Check logs for errors.${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Test data generated: $LATEST_JSON${NC}"
echo ""

# Step 3: 对标分析
echo -e "${BLUE}📈 STEP 3: Comparing with real robot data...${NC}"
python -m src.tests.analyze_sim_vs_real \
    --real "$REAL_DATA_FILE" \
    --sim "$LATEST_JSON" 2>&1 | tee "$TEST_OUTPUT_DIR/comparison.log"

echo ""
echo "╔════════════════════════════════════════════════════════════════════╗"
echo -e "${GREEN}✅ TEST SUITE COMPLETED${NC}"
echo "╚════════════════════════════════════════════════════════════════════╝"
echo ""
echo "📄 Output files:"
echo "   - $TEST_OUTPUT_DIR/diagnosis.log      (Diagnostic analysis)"
echo "   - $TEST_OUTPUT_DIR/test.log            (Raw test output)"
echo "   - $LATEST_JSON                         (Test data JSON)"
echo "   - $(dirname $LATEST_JSON)/$(basename $LATEST_JSON .json).csv  (Test data CSV)"
echo "   - $TEST_OUTPUT_DIR/comparison.log     (Real vs Sim comparison)"
echo ""
echo "📊 Next: Review the comparison.log to see if simulator matches real robot!"
echo ""
