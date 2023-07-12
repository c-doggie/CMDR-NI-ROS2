from setuptools import setup

package_name = 'py_RPi_IMU_Serial_Publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Joshua Anderson',
    maintainer_email='joshua.wei.anderson@gmail.com',
    description='A Node hosted on the main NUC that reads data from the RPi that is transmitting data and publishes it to an RPi IMU specific topic.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'py_RPi_IMU_Serial_Publisher = py_RPi_IMU_Serial_Publisher.publisher_member_function:main'
        ],
    },
)
