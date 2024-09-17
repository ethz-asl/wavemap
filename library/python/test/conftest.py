from os import path
from urllib.request import urlretrieve


def pytest_sessionstart():
    """
    Called after the Session object has been created and
    before performing collection and entering the run test loop.
    """

    # Make sure a dummy map is available for testing
    test_data_dir = path.join(path.dirname(path.abspath(__file__)), "data")
    map_url = "https://drive.google.com/uc?export=download&id=1OAgswwdJD11Ahq4x3NHQ-YElXQfkGRk3"
    map_name = "dummy_map.wvmp"
    map_storage_path = path.join(test_data_dir, map_name)
    if not path.exists(map_storage_path):
        print("Downloading dummy map for testing")
        urlretrieve(map_url, map_storage_path)
