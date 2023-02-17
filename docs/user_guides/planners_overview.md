# Planners

Most of the planners offered within our infrastructure are ready to be used after following the installation guide. However, some of them require additional steps to be taken before they can be used. All of the necessary steps and information about the planner can be found by clicking on the desired planner in the list below.

- [ROSNavRL](../packages/rosnavrl.md): Our own planner based on neural networks.
- [AllInOne](planners/all_in_one.md): a hybrid approach combining the best features of model and learning-based approaches by [Cox et al.](https://github.com/ignc-research/all-in-one-DRL-planner)
- [Dragon](planners/dragon.md): from the [BARN challenge](https://www.cs.utexas.edu/~xiao/BARN_Challenge/BARN_Challenge.html)
- [LfLH](planners/lflh.md): from the [BARN challenge](https://www.cs.utexas.edu/~xiao/BARN_Challenge/BARN_Challenge.html)
- [Trail](planners/trail.md): from the [BARN challenge](https://www.cs.utexas.edu/~xiao/BARN_Challenge/BARN_Challenge.html)
- [Applr](planners/applr.md): a hybrid approach by [Xuesu et al.](https://arxiv.org/abs/2105.07620)
- [RLCA-ROS](planners/rlca.md): a DRL-based colision avoidance approach from [Long et al.](https://github.com/Acmece/rl-collision-avoidance)
- [CADRL](planners/cadrl.md): a DRL-based colision avoidance approach from [Everett et al.](https://github.com/mit-acl/cadrl_ros)
- [SARL-Star](planners/sarl.md)
- [Crowdnav-ROS](planners/crowdnav.md): a DRL-based colision avoidance approach from [Chen et al.](https://github.com/vita-epfl/CrowdNav)
- [Cohan](planners/cohan.md): human-aware navigation from [Singamaneni et al.](https://github.com/sphanit/CoHAN_Planner/blob/master/README.md)
- TEB: a classic approach by [Rösmann et al.](https://github.com/rst-tu-dortmund/teb_local_planner)
- DWA: the standard ROS local planning approach by [Marder-Eppstein et al.](http://wiki.ros.org/dwa_local_planner)
- MPC: a classic approach by [Rösmann et al.](https://github.com/rst-tu-dortmund/teb_local_planner)

Before you start using a particular planner, have a look at the following table to see which planners are compatible with which robots.

| Planner    | Robots                      |
|------------|-----------------------------|
| TEB        | All                         |
| DWA        | All                         |
| MPC        | All                         |
| Cohan      | All                         |
| RosNavRL   | All                         |
| CADRL      | All                         |
| Dragon     | Jackal                      |
| All_in_One | Burger, Jackal, RTO, Youbot |
| LfLH       | Jackal                      |
| Applr      | Jackal                      |
| RLCA       | Burger                      |
| Trail      | Jackal                      |
| Sarl       | Burger                      |
| CrowdNav   | Burger                      |