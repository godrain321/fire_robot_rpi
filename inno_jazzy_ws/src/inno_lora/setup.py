from setuptools import find_packages, setup


package_name = 'inno_lora'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gosunwoo',
    maintainer_email='gosunwoo@example.com',
    description='LoRa sender node for the fire robot.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'lora_sender = inno_lora.lora_sender_node:main',
        ],
    },
)
