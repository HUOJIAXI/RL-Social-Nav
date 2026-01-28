import torch
import numpy as np
from crowd_sim.envs.utils.action import ActionRot, ActionXY
from crowd_nav.policy.cadrl import CADRL


class MultiHumanRL(CADRL):
    def __init__(self):
        super().__init__()

    def predict(self, state):
        """
        A base class for all methods that takes pairwise joint state as input to value network.
        The input to the value network is always of shape (batch_size, # humans, rotated joint state length)

        """
        if self.phase is None or self.device is None:
            raise AttributeError('Phase, device attributes have to be set!')
        if self.phase == 'train' and self.epsilon is None:
            raise AttributeError('Epsilon attribute has to be set in training phase')

        if self.reach_destination(state):
            return ActionXY(0, 0) if self.kinematics == 'holonomic' else ActionRot(0, 0)
        if self.action_space is None:
            self.build_action_space(state.self_state.v_pref)

        probability = np.random.random()
        if self.phase == 'train' and probability < self.epsilon:
            max_action = self.action_space[np.random.choice(len(self.action_space))]
        else:
            num_humans = len(state.human_states)
            num_actions = len(self.action_space)

            if not self.query_env:
                # Fully vectorized path - all computation on GPU
                max_action = self._predict_vectorized(state, num_actions, num_humans)
            else:
                # query_env=True path - needs environment simulation per action
                max_action = self._predict_with_env(state, num_actions, num_humans)

            if max_action is None:
                raise ValueError('Value network is not well trained. ')

        if self.phase == 'train':
            self.last_state = self.transform(state)

        return max_action

    def _predict_vectorized(self, state, num_actions, num_humans):
        """Fully vectorized prediction when query_env=False - runs entirely on GPU"""
        # Extract self state as tensor
        self_state = state.self_state
        # px, py, vx, vy, radius, gx, gy, v_pref, theta
        self_state_tensor = torch.tensor([
            self_state.px, self_state.py, self_state.vx, self_state.vy,
            self_state.radius, self_state.gx, self_state.gy, self_state.v_pref, self_state.theta
        ], dtype=torch.float32, device=self.device)

        # Extract human states as tensor: (num_humans, 5) - px, py, vx, vy, radius
        human_states_list = [[h.px, h.py, h.vx, h.vy, h.radius] for h in state.human_states]
        human_states_tensor = torch.tensor(human_states_list, dtype=torch.float32, device=self.device)

        # Extract actions as tensor: (num_actions, 2) for holonomic (vx, vy)
        if self.kinematics == 'holonomic':
            actions_tensor = torch.tensor([[a.vx, a.vy] for a in self.action_space],
                                         dtype=torch.float32, device=self.device)
        else:
            actions_tensor = torch.tensor([[a.v, a.r] for a in self.action_space],
                                         dtype=torch.float32, device=self.device)

        # Compute next self states for all actions: (num_actions, 9)
        if self.kinematics == 'holonomic':
            next_px = self_state_tensor[0] + actions_tensor[:, 0] * self.time_step
            next_py = self_state_tensor[1] + actions_tensor[:, 1] * self.time_step
            next_self_states = torch.stack([
                next_px, next_py, actions_tensor[:, 0], actions_tensor[:, 1],
                self_state_tensor[4].expand(num_actions),  # radius
                self_state_tensor[5].expand(num_actions),  # gx
                self_state_tensor[6].expand(num_actions),  # gy
                self_state_tensor[7].expand(num_actions),  # v_pref
                self_state_tensor[8].expand(num_actions),  # theta
            ], dim=1)
        else:
            next_theta = self_state_tensor[8] + actions_tensor[:, 1]
            next_vx = actions_tensor[:, 0] * torch.cos(next_theta)
            next_vy = actions_tensor[:, 0] * torch.sin(next_theta)
            next_px = self_state_tensor[0] + next_vx * self.time_step
            next_py = self_state_tensor[1] + next_vy * self.time_step
            next_self_states = torch.stack([
                next_px, next_py, next_vx, next_vy,
                self_state_tensor[4].expand(num_actions),
                self_state_tensor[5].expand(num_actions),
                self_state_tensor[6].expand(num_actions),
                self_state_tensor[7].expand(num_actions),
                next_theta,
            ], dim=1)

        # Next human states (same for all actions - linear propagation)
        # (num_humans, 5): px, py, vx, vy, radius -> next_px, next_py, vx, vy, radius
        next_human_px = human_states_tensor[:, 0] + human_states_tensor[:, 2] * self.time_step
        next_human_py = human_states_tensor[:, 1] + human_states_tensor[:, 3] * self.time_step
        next_human_states = torch.stack([
            next_human_px, next_human_py,
            human_states_tensor[:, 2], human_states_tensor[:, 3], human_states_tensor[:, 4]
        ], dim=1)  # (num_humans, 5)

        # Build joint states: (num_actions, num_humans, 14)
        # For each action, combine next_self_state with each next_human_state
        # next_self_states: (num_actions, 9), next_human_states: (num_humans, 5)
        next_self_expanded = next_self_states.unsqueeze(1).expand(-1, num_humans, -1)
        next_human_expanded = next_human_states.unsqueeze(0).expand(num_actions, -1, -1)
        joint_states = torch.cat([next_self_expanded, next_human_expanded], dim=2)

        # Reshape for rotation: (num_actions * num_humans, 14)
        joint_states_flat = joint_states.view(-1, 14)

        # Rotate on GPU
        rotated_states = self.rotate(joint_states_flat)

        # Reshape back: (num_actions, num_humans, rotated_dim)
        rotated_dim = rotated_states.shape[1]
        batched_input = rotated_states.view(num_actions, num_humans, rotated_dim)

        # Add occupancy maps if needed
        if self.with_om:
            # Build occupancy maps for the propagated human states
            next_human_states_list = [
                type(state.human_states[0])(
                    next_human_states[i, 0].item(), next_human_states[i, 1].item(),
                    next_human_states[i, 2].item(), next_human_states[i, 3].item(),
                    next_human_states[i, 4].item()
                ) for i in range(num_humans)
            ]
            occupancy_maps = self.build_occupancy_maps(next_human_states_list)
            om_expanded = occupancy_maps.unsqueeze(0).expand(num_actions, -1, -1).to(self.device)
            batched_input = torch.cat([batched_input, om_expanded], dim=2)

        # Single forward pass
        with torch.no_grad():
            all_next_values = self.model(batched_input).squeeze(-1)

        # Compute rewards vectorized on GPU
        rewards = self._compute_rewards_vectorized(next_self_states, next_human_states, num_actions, num_humans)

        # Final values
        discount = pow(self.gamma, self.time_step * state.self_state.v_pref)
        final_values = rewards + discount * all_next_values

        # Find best action
        max_idx = torch.argmax(final_values).item()
        self.action_values = final_values.cpu().tolist()

        return self.action_space[max_idx]

    def _compute_rewards_vectorized(self, next_self_states, next_human_states, num_actions, num_humans):
        """Compute rewards for all actions vectorized on GPU"""
        # next_self_states: (num_actions, 9) - px, py, vx, vy, radius, gx, gy, v_pref, theta
        # next_human_states: (num_humans, 5) - px, py, vx, vy, radius

        nav_px = next_self_states[:, 0]  # (num_actions,)
        nav_py = next_self_states[:, 1]
        nav_radius = next_self_states[:, 4]
        nav_gx = next_self_states[:, 5]
        nav_gy = next_self_states[:, 6]

        human_px = next_human_states[:, 0]  # (num_humans,)
        human_py = next_human_states[:, 1]
        human_radius = next_human_states[:, 4]

        # Distance from robot to each human for all actions: (num_actions, num_humans)
        dx = nav_px.unsqueeze(1) - human_px.unsqueeze(0)
        dy = nav_py.unsqueeze(1) - human_py.unsqueeze(0)
        dist = torch.sqrt(dx**2 + dy**2) - nav_radius.unsqueeze(1) - human_radius.unsqueeze(0)

        # Minimum distance to any human for each action
        dmin, _ = dist.min(dim=1)  # (num_actions,)

        # Check collision (any dist < 0)
        collision = (dist < 0).any(dim=1)  # (num_actions,)

        # Check reaching goal
        goal_dist = torch.sqrt((nav_px - nav_gx)**2 + (nav_py - nav_gy)**2)
        reaching_goal = goal_dist < nav_radius

        # Compute rewards
        rewards = torch.zeros(num_actions, device=self.device)
        rewards[collision] = -0.25
        rewards[~collision & reaching_goal] = 1.0
        # For non-collision, non-goal cases with dmin < 0.2
        close_mask = ~collision & ~reaching_goal & (dmin < 0.2)
        rewards[close_mask] = (dmin[close_mask] - 0.2) * 0.5 * self.time_step

        return rewards

    def _predict_with_env(self, state, num_actions, num_humans):
        """Original path when query_env=True - requires environment simulation"""
        all_raw_states = []
        all_rewards = []
        occupancy_maps = None

        for action in self.action_space:
            next_self_state = self.propagate(state.self_state, action)
            next_human_states, reward, done, info = self.env.onestep_lookahead(action)

            for next_human_state in next_human_states:
                all_raw_states.append(next_self_state + next_human_state)
            all_rewards.append(reward)

            if self.with_om and occupancy_maps is None:
                occupancy_maps = self.build_occupancy_maps(next_human_states)

        # Convert to tensor and move to GPU once
        all_states_tensor = torch.tensor(all_raw_states, dtype=torch.float32, device=self.device)

        # Batch rotate on GPU
        rotated_states = self.rotate(all_states_tensor)

        # Reshape to (num_actions, num_humans, rotated_dim)
        rotated_dim = rotated_states.shape[1]
        batched_input = rotated_states.view(num_actions, num_humans, rotated_dim)

        if self.with_om:
            om_expanded = occupancy_maps.unsqueeze(0).expand(num_actions, -1, -1).to(self.device)
            batched_input = torch.cat([batched_input, om_expanded], dim=2)

        with torch.no_grad():
            all_next_values = self.model(batched_input).squeeze(-1)

        rewards_tensor = torch.tensor(all_rewards, dtype=torch.float32, device=self.device)
        discount = pow(self.gamma, self.time_step * state.self_state.v_pref)
        final_values = rewards_tensor + discount * all_next_values

        max_idx = torch.argmax(final_values).item()
        self.action_values = final_values.cpu().tolist()

        return self.action_space[max_idx]

    def compute_reward(self, nav, humans):
        # collision detection
        dmin = float('inf')
        collision = False
        for i, human in enumerate(humans):
            dist = np.linalg.norm((nav.px - human.px, nav.py - human.py)) - nav.radius - human.radius
            if dist < 0:
                collision = True
                break
            if dist < dmin:
                dmin = dist

        # check if reaching the goal
        reaching_goal = np.linalg.norm((nav.px - nav.gx, nav.py - nav.gy)) < nav.radius
        if collision:
            reward = -0.25
        elif reaching_goal:
            reward = 1
        elif dmin < 0.2:
            reward = (dmin - 0.2) * 0.5 * self.time_step
        else:
            reward = 0

        return reward

    def transform(self, state):
        """
        Take the state passed from agent and transform it to the input of value network

        :param state:
        :return: tensor of shape (# of humans, len(state))
        """
        # Build tensor on CPU first, then transfer once
        state_tensor = torch.cat([torch.Tensor([state.self_state + human_state])
                                  for human_state in state.human_states], dim=0)
        if self.with_om:
            occupancy_maps = self.build_occupancy_maps(state.human_states)
            state_tensor = torch.cat([self.rotate(state_tensor), occupancy_maps], dim=1).to(self.device)
        else:
            state_tensor = self.rotate(state_tensor).to(self.device)
        return state_tensor

    def input_dim(self):
        return self.joint_state_dim + (self.cell_num ** 2 * self.om_channel_size if self.with_om else 0)

    def build_occupancy_maps(self, human_states):
        """

        :param human_states:
        :return: tensor of shape (# human - 1, self.cell_num ** 2)
        """
        occupancy_maps = []
        for human in human_states:
            other_humans = np.concatenate([np.array([(other_human.px, other_human.py, other_human.vx, other_human.vy)])
                                         for other_human in human_states if other_human != human], axis=0)
            other_px = other_humans[:, 0] - human.px
            other_py = other_humans[:, 1] - human.py
            # new x-axis is in the direction of human's velocity
            human_velocity_angle = np.arctan2(human.vy, human.vx)
            other_human_orientation = np.arctan2(other_py, other_px)
            rotation = other_human_orientation - human_velocity_angle
            distance = np.linalg.norm([other_px, other_py], axis=0)
            other_px = np.cos(rotation) * distance
            other_py = np.sin(rotation) * distance

            # compute indices of humans in the grid
            other_x_index = np.floor(other_px / self.cell_size + self.cell_num / 2)
            other_y_index = np.floor(other_py / self.cell_size + self.cell_num / 2)
            other_x_index[other_x_index < 0] = float('-inf')
            other_x_index[other_x_index >= self.cell_num] = float('-inf')
            other_y_index[other_y_index < 0] = float('-inf')
            other_y_index[other_y_index >= self.cell_num] = float('-inf')
            grid_indices = self.cell_num * other_y_index + other_x_index
            occupancy_map = np.isin(range(self.cell_num ** 2), grid_indices)
            if self.om_channel_size == 1:
                occupancy_maps.append([occupancy_map.astype(int)])
            else:
                # calculate relative velocity for other agents
                other_human_velocity_angles = np.arctan2(other_humans[:, 3], other_humans[:, 2])
                rotation = other_human_velocity_angles - human_velocity_angle
                speed = np.linalg.norm(other_humans[:, 2:4], axis=1)
                other_vx = np.cos(rotation) * speed
                other_vy = np.sin(rotation) * speed
                dm = [list() for _ in range(self.cell_num ** 2 * self.om_channel_size)]
                for i, index in np.ndenumerate(grid_indices):
                    if index in range(self.cell_num ** 2):
                        if self.om_channel_size == 2:
                            dm[2 * int(index)].append(other_vx[i])
                            dm[2 * int(index) + 1].append(other_vy[i])
                        elif self.om_channel_size == 3:
                            dm[3 * int(index)].append(1)
                            dm[3 * int(index) + 1].append(other_vx[i])
                            dm[3 * int(index) + 2].append(other_vy[i])
                        else:
                            raise NotImplementedError
                for i, cell in enumerate(dm):
                    dm[i] = sum(dm[i]) / len(dm[i]) if len(dm[i]) != 0 else 0
                occupancy_maps.append([dm])

        return torch.from_numpy(np.concatenate(occupancy_maps, axis=0)).float()

