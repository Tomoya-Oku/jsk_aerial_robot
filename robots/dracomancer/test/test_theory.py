#!/usr/bin/env python3
"""Thesis equations: wrench reference, compensation, and haptic work."""
from pathlib import Path
import sys
import unittest

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'src'))
from dracomancer.contacts import ContactModel, anchor_position, required_wrench
from dracomancer.haptics import ContactFeedback, SafetyFeedback
from dracomancer.shape import finite_jacobian


class TheoryTests(unittest.TestCase):
    def test_contact_equilibrium_includes_lever_arm(self):
        np.testing.assert_allclose(required_wrench(2., [0, 0, -9.81], [0]*6, [1, 0, 0]),
                                   [0, 0, 19.62, 0, 0, 0])
        np.testing.assert_allclose(required_wrench(2., [0, 0, -9.81], [0, 3, 0, 0, 0, 2], [1, 0, 0]),
                                   [0, -3, 19.62, 0, 0, -5])

    def test_anchor_compensation_keeps_world_point(self):
        rotation = np.array([[0., -1, 0], [1., 0, 0], [0., 0, 1]])
        reference = np.array([2., 3, 4])
        for relative in [np.array([.2, 0, 0]), np.array([0, .5, .1])]:
            command = anchor_position(reference, rotation, relative)
            np.testing.assert_allclose(command + rotation @ relative, reference)
        with self.assertRaises(ValueError):
            anchor_position(reference, np.ones((3, 3)), [0, 0, 0])

    def test_contact_at_fixed_anchor_cannot_reflect_force(self):
        model = ContactModel('''<robot name="test">
          <link name="fc"><inertial><mass value="1"/></inertial></link>
          <link name="link1"><inertial><mass value="1"/><origin xyz="1 0 0"/></inertial></link>
          <joint name="bend" type="revolute"><parent link="fc"/><child link="link1"/>
            <origin xyz="1 0 0"/><axis xyz="0 0 1"/></joint></robot>''')
        position = {'bend': .4}
        offset = [.3, 0, 0]
        jacobian = model.jacobian(position, ['bend'], offset, {'link': 'link1', 'xyz': offset})
        np.testing.assert_allclose(jacobian[:3], 0., atol=1e-10)
        self.assertGreater(np.linalg.norm(jacobian[3:]), .9)
        torque = ContactFeedback(1, 1).compute([2, 3, 4, 0, 0, 0], jacobian, np.ones((1, 1)))
        np.testing.assert_allclose(torque, 0., atol=1e-10)

    def test_safety_torque_is_negative_potential_gradient(self):
        # q_cmd and the previous command in the mapping remain frozen.
        mapping = np.arange(84., dtype=float).reshape(6, 14)/100
        q_h = np.linspace(-.2, .2, 14)
        q_cmd, stiffness, gain = np.full(6, .1), np.arange(1., 7.), .7
        def potential(q):
            error = mapping @ q - q_cmd
            return np.array([.5 * gain * np.dot(error, stiffness * error)])
        expected = -finite_jacobian(potential, q_h).ravel()
        actual = SafetyFeedback(stiffness, np.zeros(14), gain).compute(mapping @ q_h-q_cmd, mapping, np.zeros(14))
        np.testing.assert_allclose(actual, expected, atol=1e-9)

    def test_negative_haptic_energy_parameters_are_rejected(self):
        with self.assertRaises(ValueError):
            SafetyFeedback([-1.], [0.])
        with self.assertRaises(ValueError):
            ContactFeedback(6, 14, gain=-1.)

    def test_opposing_grasp_forces_remain_in_joint_feedback(self):
        model=ContactModel('''<robot name="grasp">
          <link name="fc"><inertial><mass value="1"/></inertial></link>
          <link name="left"><inertial><mass value="1"/></inertial></link>
          <link name="right"><inertial><mass value="1"/></inertial></link>
          <joint name="l" type="prismatic"><parent link="fc"/><child link="left"/><axis xyz="-1 0 0"/></joint>
          <joint name="r" type="prismatic"><parent link="fc"/><child link="right"/><axis xyz="1 0 0"/></joint>
          </robot>''',tip='left')
        values={'l':.5,'r':.5}
        force=np.array([1.,0,0,0,0,0])
        mapper=ContactFeedback(2,2)
        left=mapper.compute(force,model.jacobian(values,['l','r'],link='left'),np.eye(2))
        right=mapper.compute(-force,model.jacobian(values,['l','r'],link='right'),np.eye(2))
        np.testing.assert_allclose(force-force,0.)
        np.testing.assert_allclose(left+right,[-1.,-1.],atol=1e-8)

    def test_view_plane_neutral_switch_and_rotation_integration(self):
        from dracomancer.navigation import ViewControl
        view=ViewControl([0,0,0],linear=[1,1],angular=[1,1])
        v,w=view.commands([1,0,0],np.eye(3),[1,0])
        np.testing.assert_allclose(v,[0,-1,0]);np.testing.assert_allclose(w,0.)
        view.button(True)
        v,w=view.commands([1,0,0],np.eye(3),[1,0])
        np.testing.assert_allclose(v,0.);np.testing.assert_allclose(w,0.)
        view.commands([1,0,0],np.eye(3),[0,0])
        self.assertTrue(view.rotate_mode)
        v,w=view.commands([1,0,0],np.eye(3),[1,0])
        np.testing.assert_allclose(v,0.);np.testing.assert_allclose(w,[0,0,1.])
        angles=view.integrate_rotation(np.eye(3),w,.1,[[-1,1],[-1,1],[-3,3]])
        np.testing.assert_allclose(angles,[0,0,.1],atol=1e-12)

    def test_view_vertical_singularity_is_continuous(self):
        from dracomancer.navigation import ViewControl
        view=ViewControl([0,0,1])
        first=view.basis([.00001,0,0],np.eye(3))
        second=view.basis([-.00001,0,0],np.eye(3))
        self.assertGreater(np.dot(first[:,0],second[:,0]),.999)


if __name__ == '__main__':
    unittest.main()
