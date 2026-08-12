import pytest
from pytest_embedded import Dut


@pytest.mark.target('esp32s3')
@pytest.mark.env('esp32s3')
@pytest.mark.parametrize(
    'config',
    ['mn5q8_en', 'mn6_cn', 'mn7_en'],
)
def test_multinet_s3(dut: Dut) -> None:
    dut.run_all_single_board_cases(group='mn')


@pytest.mark.target('esp32p4')
@pytest.mark.env('esp32p4')
@pytest.mark.parametrize(
    'config',
    ['p4_mn7_en', 'p4_mn7_cn'],
)
def test_multinet_p4(dut: Dut) -> None:
    dut.run_all_single_board_cases(group='mn')
