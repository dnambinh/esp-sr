import pytest
from pytest_embedded import Dut


@pytest.mark.target('esp32s3')
@pytest.mark.env('esp32s3')
@pytest.mark.parametrize('config', ['afe'])
def test_afe(dut: Dut) -> None:
    dut.run_all_single_board_cases(group='afe', timeout=3600)


@pytest.mark.target('esp32p4')
@pytest.mark.env('esp32p4')
@pytest.mark.parametrize(
    'config',
    ['p4_afe'],
)
def test_afe_p4(dut: Dut) -> None:
    dut.run_all_single_board_cases(group='afe', timeout=3600)
