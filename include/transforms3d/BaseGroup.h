/**
 * @file     BaseGroup.h
 * @author   Wang Zhen (wangzhen9623@163.com)
 * @brief
 * @version  0.1
 * @date     2023-07-13
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef BASEGROUP_H
#define BASEGROUP_H

#include <deque>
#include <iostream>
#include <map>
#include <memory>
#include <vector>

#include "BaseGroupChild.h"
namespace transforms3d {

class BasePath {
 public:
  std::string name;
  std::vector<std::string> path;
  BasePath(const std::string& name, const std::vector<std::string>& path)
      : name(name), path(path) {}
  virtual ~BasePath() {}
};

class BaseGroup {
 protected:
  std::shared_ptr<IGroupChildFactory> m_child_factory;
  std::map<std::string, std::vector<std::shared_ptr<BaseGroupChild>>> m_childs;

 public:
  BaseGroup(std::shared_ptr<IGroupChildFactory> child_factory)
      : m_child_factory(child_factory) {}

  /**
   * @brief: 将存储的节点数据按照字符串形式输出
   * @param {*} 无
   * @return {std::string}  字符串
   */
  virtual std::string toString() {
    std::string str;
    typename std::map<std::string,
                      std::vector<std::shared_ptr<BaseGroupChild>>>::iterator
        iter;
    for (iter = m_childs.begin(); iter != m_childs.end(); iter++) {
      // cout << "node:" << iter->first << endl;
      str += "node:" + iter->first + "\n";
      for (int i = 0; i < iter->second.size(); i++) {
        str += "\tchild: " + iter->second[i]->name;
      }
      std::cout << std::endl;
      str += "\n";
    }
    return str;
  };

  /**
   * @brief: 查找两个节点之间的路径
   * @param {const std::string&} start  开始节点名字
   * @param {const std::string&} end  结束节点名字
   * @return {std::vector<std::string> }  路径数组
   */
  virtual std::vector<std::string> findPath(const std::string &start,
                                            const std::string &end) {
    std::deque<BasePath> que;
    std::vector<std::string> researched = {start};
    if (start == end) return researched;
    for (auto c : m_childs[start]) {
      BasePath path = {c->name, {start, c->name}};
      que.push_back(path);
    }
    while (!que.empty()) {
      BasePath path = que.front();
      // cout<<"-----------------------------"<<endl;;
      // cout<<path.name<<":";
      // for(std::string p:path.path){
      //     cout<<"   "<<p;
      // }
      // cout<<"researched"<<":";
      // for(std::string r:researched){
      //     cout<<"   "<<r;
      // }
      // cout<<endl;
      // cout<<"before quesize:"<<que.size()<<endl;

      que.pop_front();

      if (std::find(researched.begin(), researched.end(), path.name) ==
          researched.end()) {
        // cout<<"not find path.name"<<endl;
        if (path.name == end) {
          // cout<<"fing node"<<endl;
          return path.path;
        } else {
          for (auto c : m_childs[path.name]) {
            // cout<<"prepare add "<<c.name<<endl;
            if (std::find(researched.begin(), researched.end(), c->name) ==
                researched.end()) {
              std::vector<std::string> paths = {c->name};
              paths.insert(paths.begin(), path.path.begin(), path.path.end());
              BasePath temPath = {c->name, paths};
              que.push_back(temPath);
              // cout<<"added "<<c.name<<endl;
            }
          }
        }
      }
      // cout<<"quesize:"<<que.size()<<endl;
    }

    return researched;
  };

  /**
   * @brief: 添加一个节点
   * @param {const std::string&} parent  父节点名字
   * @param {const std::string&} child  子节点名字
   * @return {bool}  是否添加成功
   */
  virtual bool addChild(const std::string &parent, const std::string &child) {
    bool has_parent_node = true, has_child_node = true;
    if (m_childs.find(parent) == m_childs.end()) has_parent_node = false;
    if (m_childs.find(child) == m_childs.end()) has_child_node = false;

    // 分情况判断
    if (has_parent_node) {
      std::vector<std::shared_ptr<BaseGroupChild>> childs = m_childs[parent];
      bool has_child = false;
      for (int i = 0; i < childs.size(); i++) {
        if (childs[i]->name == child) {
          has_child = true;
          // NOTE: do someting 这里应该是一个多态操作，但是我不知道怎么写
          // m_childs[parent][i].t = matrix;
        }
      }
      if (!has_child) {
        // 没有这个孩子，添加孩子
        auto new_child = m_child_factory->create(child);
        // new_child->name = child;
        // 多态操作
        // new_child.t = matrix;
        m_childs[parent].push_back(new_child);
      }
    } else {
      // 添加一个节点
      auto new_child = m_child_factory->create(child);
      // new_child->name = child;
      // new_child.t = matrix;
      std::vector<std::shared_ptr<BaseGroupChild>> new_childs;
      new_childs.push_back(new_child);
      //添加一个父节点
      m_childs[parent] = new_childs;
    }

    // 分情况判断
    if (has_child_node) {
      std::vector<std::shared_ptr<BaseGroupChild>> childs = m_childs[child];
      bool has_child = false;
      for (int i = 0; i < childs.size(); i++) {
        if (childs[i]->name == child) {
          has_child = true;
          // // 更新已有的矩阵
          // m_childs[child][i].t = matrix;
        }
      }
      if (!has_child) {
        // 没有这个孩子，添加孩子
        auto new_child = m_child_factory->create(child);

        new_child->name = parent;
        // new_child.t = matrix.inverse();
        m_childs[child].push_back(new_child);
      }
    } else {
      // 添加一个节点
      auto new_child = m_child_factory->create(child);

      new_child->name = parent;
      // new_child.t = matrix.inverse();
      std::vector<std::shared_ptr<BaseGroupChild>> new_childs;
      new_childs.push_back(new_child);
      m_childs[child] = new_childs;
    }

    return true;
  }
  virtual ~BaseGroup() {}
};

}  // namespace CDC

#endif  // BASEGROUP_H
