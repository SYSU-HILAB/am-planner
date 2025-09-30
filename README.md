# Whole-Body Integrated Motion Planning for Aerial Manipulators

<p align="center">
  <a href="https://arxiv.org/abs/2501.06493">
    <img src="https://img.shields.io/badge/Paper-arXiv-red.svg" alt="Paper">
  </a>
  <a href="https://www.bilibili.com/video/BV1pxcHebEqr?vd_source=3bacea077de525a48604ad1df69038ed">
    <img src="https://img.shields.io/badge/Video-Bilibili-blue.svg" alt="Video">
  </a>
  <a href="https://am-planner.github.io">
    <img src="https://img.shields.io/badge/Project-Page-green.svg" alt="Project Page">
  </a>
</p>

<div align="center">
    <a href="https://www.bilibili.com/video/BV1pxcHebEqr/?spm_id_from=333.1387.homepage.video_card.click&vd_source=d450cee11e9b588cf172b8ee1b211101" target="_blank">
    <img src="./assets/cover.JPG" width="50%" />
    </a>
</div>


## News🔥


**[2025.9.26]** Paper accepted by *IEEE Transactions on Robotics (T-RO)* 🚀🎉

## Main Contributors

[Weiliang Deng（邓伟亮）](https://dwl2021.github.io), [Hongming Chen（陈鸿铭）](https://xiaodao-chen.github.io) 

## Overview

AM-Planner is a novel whole-body integrated motion planning framework for quadrotor-based aerial manipulators that enables versatile manipulation capabilities through flexible waypoint constraints. Our framework simultaneously optimizes trajectories for both the quadrotor and manipulator to achieve coordinated whole-body motion across a wide range of manipulation tasks.

### Key Features

- **Whole-body Planning**: Simultaneous optimization of quadrotor and manipulator trajectories
- **Flexible Waypoint Constraints**: Selective specification of position, velocity, or orientation requirements
- **IL-Guided Optimization**: Imitation learning to overcome poor local optima in challenging scenarios
- **Collision-Free Manipulation**: Novel varying ellipsoid method for dynamic collision avoidance

## Code Availability

The code will be made available before publication.

## Paper and Citation

If you use this work in your research, please cite our paper:

```bibtex
@misc{deng2025whole,
      title={Whole-Body Integrated Motion Planning for Aerial Manipulators}, 
      author={Weiliang Deng and Hongming Chen and Biyu Ye and Haoran Chen and Ximin Lyu},
      year={2025},
      eprint={2501.06493},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2501.06493}, 
}

@INPROCEEDINGS{chen_ndob-based_nodate,
  author={Chen, Hongming and Ye, Biyu and Liang, Xianqi and Deng, Weiliang and Lyu, Ximin},
  booktitle={2025 IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={NDOB-Based Control of a UAV with Delta-Arm Considering Manipulator Dynamics}, 
  year={2025},
  pages={7505-7511},
  keywords={Robust control;Couplings;Accuracy;Grasping;Three-dimensional printing;Autonomous aerial vehicles;End effectors;Disturbance observers;Robotics and automation;Manipulator dynamics},
  doi={10.1109/ICRA55743.2025.11127414}}

@misc{chen2025aerial,
      title={Aerial Grasping via Maximizing Delta-Arm Workspace Utilization}, 
      author={Haoran Chen and Weiliang Deng and Biyu Ye and Yifan Xiong and Zongliang Pan and Ximin Lyu},
      year={2025},
      eprint={2506.15539},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2506.15539}, 
}
```

## Acknowledgments

We also thank Weichen Lyu (THU), Jiarui Ouyang (HKUST), and Binpei Luo (SYSU) for their valuable assistance and continuous encouragement throughout this project.

## Contact

For questions or issues, please open an issue on GitHub or contact us at dengwl.2021 [at] gmail.com.