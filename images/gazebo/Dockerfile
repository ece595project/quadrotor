FROM gazebo:gzserver8

RUN ["bash", "-c", "source /usr/share/gazebo-8/setup.sh"]

RUN apt-get update -q && apt-get install -qy curl && \
  curl -o double_pendulum.sdf http://models.gazebosim.org/double_pendulum_with_base/model-1_4.sdf

CMD ["gzserver"]
# CMD gz model --model-name double_pendulum --spawn-file double_pendulum.sdf
