from setuptools import find_packages, setup

package_name = 'main'

setup(
    name=package_name,
    version='0.0.0',
    # 搜索当前目录（main 功能包目录）下的包
    packages=find_packages(where='.'),  
    # 无需额外映射，或设为当前目录
    package_dir={'': '.'},  
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='sunrise@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_ctrl = main.main_ctrl:main',
            'pid_test = main.pid_test:main',
            'main = main.main:main',
            'main_stop = main.main_stop:main',
            'task_state_machine = main.task_state_machine:main',
            'test_45 = main.test_45:main',
            'test_45_2 = main.test_45_2:main',
        ],
    },
)
