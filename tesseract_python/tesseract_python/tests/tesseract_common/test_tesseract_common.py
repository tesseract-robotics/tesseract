from tesseract import tesseract_common

def test_status_code():
    # Test that status codes can be created

    status_code = tesseract_common.StatusCode(100, tesseract_common.GeneralStatusCategory())

    print(status_code)

def test_bytes_resource():

    my_bytes = bytearray([10,57,92,56,92,46,92,127])
    my_bytes_url = "file:///test_bytes.bin"
    bytes_resource = tesseract_common.BytesResource(my_bytes_url,my_bytes)
    my_bytes_ret = bytes_resource.getResourceContents()
    assert(len(my_bytes_ret) == len(my_bytes))
    assert(my_bytes == bytearray(my_bytes_ret))
    assert(my_bytes_url == bytes_resource.getUrl())