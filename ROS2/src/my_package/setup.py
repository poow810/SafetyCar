from setuptools import setup

package_name = 'my_package'

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
    py_modules=['my_package.dynamic_window_approach'],
    zip_safe=True,
    maintainer='SSAFY',
    maintainer_email='seo6893@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_package.my_node:main',
            'talker = my_package.publisher_member_function:main',
            'listner = my_package.subscriber_member_function:main',
            # 'odom = my_package.run_localization:main',
            'odom = my_package.odometry:main',
            'ftc = my_package.follow_the_carrot:main',
            'load_map = my_package.load_map:main',
            'a_star_global = my_package.a_star:main',
            'a_star_local = my_package.a_star_local_path:main',
            'mapping = my_package.run_mapping:main',
            'dwa_local = my_package.dwa_local_path:main',
            'test = my_package.data_socket:main',
            # 'dwa = my_package.dynamic_window_approach:main',
        ],
    },
)
